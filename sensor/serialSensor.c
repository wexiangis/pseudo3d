/*
 *  接收来自串口的传感器数据
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

// --------------------------------- serial

#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

#define TIMEOUT 1   /* read operation timeout 1s = TIMEOUT/10 */
#define MIN_LEN 128 /* the min len datas */
#define DEV_NAME_LEN 11
#define SERIAL_ATTR_BAUD 115200
#define SERIAL_ATTR_DATABITS 8
#define SERIAL_ATTR_STOPBITS 1
#define SERIAL_ATTR_PARITY 'n'
#define SERIAL_MODE_NORMAL 0
#define SERIAL_MODE_485 1

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

typedef struct
{
    unsigned int lable;
    unsigned int baudrate;
} SERIAL_BAUD_ST;

typedef struct
{
    char parity;
    unsigned int baud;
    unsigned int databits;
    unsigned int stopbits;
} SERIAL_ATTR_ST;

static SERIAL_BAUD_ST g_attr_baud[] = {
    {921600, B921600},
    {460800, B460800},
    {230400, B230400},
    {115200, B115200},
    {57600, B57600},
    {38400, B38400},
    {19200, B19200},
    {9600, B9600},
    {4800, B4800},
    {2400, B2400},
    {1800, B1800},
    {1200, B1200},
};

static int attr_baud_set(int fd, unsigned int baud)
{
    int i;
    int ret = 0;
    struct termios option;
    /* get old serial attribute */
    memset(&option, 0, sizeof(option));
    if (0 != tcgetattr(fd, &option))
    {
        printf("tcgetattr failed.\n");
        return -1;
    }
    for (i = 0; i < ARRAY_SIZE(g_attr_baud); i++)
    {
        if (baud == g_attr_baud[i].lable)
        {
            ret = tcflush(fd, TCIOFLUSH);
            if (0 != ret)
            {
                printf("tcflush failed.\n");
                break;
            }
            ret = cfsetispeed(&option, g_attr_baud[i].baudrate);
            if (0 != ret)
            {
                printf("cfsetispeed failed.\n");
                ret = -1;
                break;
            }
            ret = cfsetospeed(&option, g_attr_baud[i].baudrate);
            if (0 != ret)
            {
                printf("cfsetospeed failed.\n");
                ret = -1;
                break;
            }
            ret = tcsetattr(fd, TCSANOW, &option);
            if (0 != ret)
            {
                printf("tcsetattr failed.\n");
                ret = -1;
                break;
            }
            ret = tcflush(fd, TCIOFLUSH);
            if (0 != ret)
            {
                printf("tcflush failed.\n");
                break;
            }
        }
    }
    return ret;
}

static int attr_other_set(int fd, SERIAL_ATTR_ST *serial_attr)
{
    struct termios option;
    /* get old serial attribute */
    memset(&option, 0, sizeof(option));
    if (0 != tcgetattr(fd, &option))
    {
        printf("tcgetattr failed.\n");
        return -1;
    }
    option.c_iflag = CLOCAL | CREAD;
    /* set datas size */
    option.c_cflag &= ~CSIZE;
    option.c_iflag = 0;
    switch (serial_attr->databits)
    {
    case 7:
        option.c_cflag |= CS7;
        break;
    case 8:
        option.c_cflag |= CS8;
        break;
    default:
        printf("invalid argument, unsupport datas size.\n");
        return -1;
    }
    /* set parity */
    switch (serial_attr->parity)
    {
    case 'n':
    case 'N':
        option.c_cflag &= ~PARENB;
        option.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O':
        option.c_cflag |= (PARODD | PARENB);
        option.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E':
        option.c_cflag |= PARENB;
        option.c_cflag &= ~PARODD;
        option.c_iflag |= INPCK;
        break;
    case 's':
    case 'S':
        option.c_cflag &= ~PARENB;
        option.c_cflag &= ~CSTOPB;
        break;
    default:
        printf("invalid argument, unsupport parity type.\n");
        return -1;
    }
    /* set stop bits  */
    switch (serial_attr->stopbits)
    {
    case 1:
        option.c_cflag &= ~CSTOPB;
        break;
    case 2:
        option.c_cflag |= CSTOPB;
        break;
    default:
        printf("invalid argument, unsupport stop bits.\n");
        return -1;
    }
    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_cc[VTIME] = TIMEOUT;
    option.c_cc[VMIN] = MIN_LEN;
    if (0 != tcflush(fd, TCIFLUSH))
    {
        printf("tcflush failed.\n");
        return -1;
    }
    if (0 != tcsetattr(fd, TCSANOW, &option))
    {
        printf("tcsetattr failed.\n");
        return -1;
    }
    return 0;
}

static int attr_set(int fd, SERIAL_ATTR_ST *serial_attr)
{
    int ret = 0;
    if (NULL == serial_attr)
    {
        printf("invalid argument.\n");
        return -1;
    }
    if (0 == ret)
    {
        ret = attr_baud_set(fd, serial_attr->baud);
        if (0 == ret)
        {
            ret = attr_other_set(fd, serial_attr);
        }
    }
    return ret;
}

// --------------------------------- sensor
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "mpu6050.h"
#include "serialSensor.h"

typedef struct
{
    int fd;
    SERIAL_ATTR_ST serial_attr;
    pthread_t th;
    //当前读写位置
    uint32_t buff_r, buff_w;
    //缓冲区
    uint8_t buff[SERIAL_CIRCLE_BUFF_LEN];
} Serial_Sensor;

static Serial_Sensor *serial_sensor = NULL;

void serialSensor_thread(void *argv)
{
    Serial_Sensor *ss = (Serial_Sensor *)argv;
    size_t ret = 0;
    uint8_t buff[1024];
    printf("serialSensor: thread start \r\n");
    while (ss->fd > 0)
    {
        ret = read(ss->fd, buff, sizeof(buff));//阻塞读
        if (ret > 0)
        {
            if (ss->buff_w + ret < SERIAL_CIRCLE_BUFF_LEN)
            {
                memcpy(&ss->buff[ss->buff_w], buff, ret);
                ss->buff_w += ret;
            }
            else
            {
                memcpy(&ss->buff[ss->buff_w], buff, SERIAL_CIRCLE_BUFF_LEN - ss->buff_w);
                ret -= SERIAL_CIRCLE_BUFF_LEN - ss->buff_w;
                if (ret > 0)
                    memcpy(&ss->buff[ss->buff_w], buff, ret);
                ss->buff_w = ret;
            }
        }
        else if (ret < 0)
            break;
    }
    close(ss->fd);
    ss->fd = 0;
    printf("serialSensor: thread exit \r\n");
}

int _serialSensor_getPkg(Serial_Sensor *ss, uint8_t pkg[46])
{
    //读写位置暂停
    uint32_t buff_r;
    uint32_t buff_w;
    uint32_t count;
    uint8_t head[4] = {0};
    uint8_t crc;
    //定格写位置
    buff_w = ss->buff_w;
// try_again:
    count = 0;
    buff_r = ss->buff_r;
    while (buff_r != buff_w && count < 46)
    {
        if (count == 0)
        {
            //滚雪球
            head[0] = head[1];
            head[1] = head[2];
            head[2] = head[3];
            head[3] = ss->buff[buff_r];
            //凑够这个特殊顺序,召唤神龙
            if (head[0] == 0x7E && head[1] == 0x7E && head[2] == 0x0C)
            {
                //开始收包
                count = 4;
                memcpy(pkg, head, 4);
            }
        }
        else
            pkg[count++] = ss->buff[buff_r];
        //循环
        buff_r += 1;
        if (buff_r >= SERIAL_CIRCLE_BUFF_LEN)
            buff_r = 0;
    }
    //校验
    if (count == 46)
    {
        //更新读标志
        ss->buff_r = buff_r;
        //尾标志不对
        if (pkg[45] != 0xBF)
        {
            printf(" tail error : state code %02X \r\n", pkg[3]);
            // goto try_again;
        }
        //校验
        for (crc = 0, count = 0; count < 44; count++)
            crc ^= pkg[count];
        if (crc != pkg[44])
        {
            printf(" crc error : state code %02X \r\n", pkg[3]);
            // goto try_again;
        }
        // printf("find pkg / %d \r\n", *((uint32_t *)&pkg[4]));
        //成功
        return 0;
    }
    //不满一包
    return 1;
}

int serialSensor_decode(Serial_Sensor *ss, float *gyro, float *accel)
{
    float vFloat[10];
    uint8_t pkg[46];
    //从循环缓冲区中取一包数据
    if (_serialSensor_getPkg(ss, pkg) != 0)
        return 1;
    memcpy(vFloat, &pkg[8], 9 * 4);
    //包数据转参数
    if (accel)
    {
        accel[0] = vFloat[0];
        accel[1] = vFloat[1];
        accel[2] = vFloat[2];
    }
    if (gyro)
    {
        gyro[0] = vFloat[3];
        gyro[1] = vFloat[4];
        gyro[2] = vFloat[5];
    }

    // printf(" %8.6f %8.6f %8.6f // %8.6f %8.6f %8.6f\r\n", 
    //     vFloat[0], vFloat[1], vFloat[2], 
    //     vFloat[3], vFloat[4], vFloat[5]);

    // printf(" %8.6f %8.6f %8.6f // %8.6f %8.6f %8.6f // %8.6f %8.6f %8.6f \r\n", 
    //     vFloat[0], vFloat[1], vFloat[2], 
    //     vFloat[3], vFloat[4], vFloat[5], 
    //     vFloat[6], vFloat[7], vFloat[8]);

    return 0;
}

/*
 *  返回0正常
 */
int serialSensor_get(float *gyro, float *accel)
{
    //打开串口
    if (!serial_sensor || serial_sensor->fd < 1)
    {
        if (!serial_sensor)
            serial_sensor = (Serial_Sensor *)calloc(1, sizeof(Serial_Sensor));
        serial_sensor->fd = open(SERIAL_DEV, O_RDONLY);
        if (serial_sensor->fd < 1)
            return 1;
        memset(&serial_sensor->serial_attr, 0, sizeof(serial_sensor->serial_attr));
        serial_sensor->serial_attr.baud = SERIAL_BAUND;
        serial_sensor->serial_attr.databits = 8;
        serial_sensor->serial_attr.stopbits = 1;
        serial_sensor->serial_attr.parity = 'n';
        attr_set(serial_sensor->fd, &serial_sensor->serial_attr);
        //开线程
        pthread_create(&serial_sensor->th, NULL, (void *)serialSensor_thread, (void *)serial_sensor);
        return 1;
    }
    //从循环缓冲区解析数据
    return serialSensor_decode(serial_sensor, gyro, accel);
}
