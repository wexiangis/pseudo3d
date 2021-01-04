/*
 *  接收来自串口的传感器数据
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
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

#include "serialSensor.h"

typedef struct
{
    int fd;
    SERIAL_ATTR_ST serial_attr;
    pthread_t th;
    //当前读写位置
    uint32_t buff_r, buff_w;
    //缓冲区
    char temp[SERIAL_CIRCLE_BUFF_POINT];//温度 -128~127

    uint32_t timeStamp[SERIAL_CIRCLE_BUFF_POINT];

    float accX[SERIAL_CIRCLE_BUFF_POINT];
    float accY[SERIAL_CIRCLE_BUFF_POINT];
    float accZ[SERIAL_CIRCLE_BUFF_POINT];

    float gyrX[SERIAL_CIRCLE_BUFF_POINT];
    float gyrY[SERIAL_CIRCLE_BUFF_POINT];
    float gyrZ[SERIAL_CIRCLE_BUFF_POINT];

    float compassX[SERIAL_CIRCLE_BUFF_POINT];
    float compassY[SERIAL_CIRCLE_BUFF_POINT];
    float compassZ[SERIAL_CIRCLE_BUFF_POINT];

    float angleAddX[SERIAL_CIRCLE_BUFF_POINT];
    float angleAddY[SERIAL_CIRCLE_BUFF_POINT];
    float angleAddZ[SERIAL_CIRCLE_BUFF_POINT];

    float speedAddX[SERIAL_CIRCLE_BUFF_POINT];
    float speedAddY[SERIAL_CIRCLE_BUFF_POINT];
    float speedAddZ[SERIAL_CIRCLE_BUFF_POINT];
} Serial_Sensor;

static Serial_Sensor *serial_sensor = NULL;

static bool _crcCheck(uint8_t *buff)
{
    int i;
    uint8_t crc = 0;
    for (i = 0; i < 70 - 2; i++)
        crc ^= buff[i];
    if (crc == buff[i])
        return true;
    printf(" serialSensor: crc error\r\n");
    return false;
}

void serialSensor_thread(void *argv)
{
    Serial_Sensor *ss = (Serial_Sensor *)argv;
    size_t ret = 0, addr = 0, count;
    uint8_t buff[1024];
    uint8_t *pBuff;
    float vFloat[15];

    // printf("serialSensor: thread start \r\n");

    while (ss->fd > 0)
    {
        ret = read(ss->fd, &buff[addr], sizeof(buff) - addr);//阻塞读
        if (ret > 0)
        {
            ret += addr;
            addr = 0;

            // printf("ret/%d %02X %02X %02X %02X \r\n",
            //      ret, buff[0], buff[1], buff[2], buff[3]);

            pBuff = (uint8_t *)buff;
            while(ret >= 70)
            {
                if (pBuff[0] == 0x7E && pBuff[1] == 0x7E && pBuff[2] == 0x0C)
                {
                    _crcCheck(pBuff);

                    ss->temp[ss->buff_w] = (char)pBuff[3];

                    ss->timeStamp[ss->buff_w] = *((uint32_t *)&pBuff[4]);

                    //直接指针传递可能地址不是4的倍数
                    memcpy(vFloat, &pBuff[8], sizeof(float) * 15);

                    ss->accY[ss->buff_w] = vFloat[0];
                    ss->accX[ss->buff_w] = vFloat[1];
                    ss->accZ[ss->buff_w] = vFloat[2];

                    ss->gyrY[ss->buff_w] = vFloat[3];
                    ss->gyrX[ss->buff_w] = vFloat[4];
                    ss->gyrZ[ss->buff_w] = vFloat[5];

                    ss->compassX[ss->buff_w] = vFloat[6];
                    ss->compassY[ss->buff_w] = vFloat[7];
                    ss->compassZ[ss->buff_w] = vFloat[8];

                    // printf("%02X %02X %02X %02X %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %06dms\r\n",
                    //     pBuff[0], pBuff[1], pBuff[2], pBuff[3],
                    //     ss->accX[ss->buff_w],
                    //     ss->accY[ss->buff_w],
                    //     ss->accZ[ss->buff_w],
                    //     ss->gyrY[ss->buff_w],
                    //     ss->gyrY[ss->buff_w],
                    //     ss->gyrY[ss->buff_w],
                    //     ss->compassX[ss->buff_w],
                    //     ss->compassY[ss->buff_w],
                    //     ss->compassZ[ss->buff_w],
                    //     ss->timeStamp[ss->buff_w]);

                    ss->angleAddX[ss->buff_w] = vFloat[9];
                    ss->angleAddY[ss->buff_w] = vFloat[10];
                    ss->angleAddZ[ss->buff_w] = vFloat[11];

                    ss->speedAddX[ss->buff_w] = vFloat[12];
                    ss->speedAddY[ss->buff_w] = vFloat[13];
                    ss->speedAddZ[ss->buff_w] = vFloat[14];

                    // printf("aa %8.4f %8.4f %8.4f sa %8.4f %8.4f %8.4f temp %d\r\n",
                    //     ss->angleAddX[ss->buff_w],
                    //     ss->angleAddY[ss->buff_w],
                    //     ss->angleAddZ[ss->buff_w],
                    //     ss->speedAddX[ss->buff_w],
                    //     ss->speedAddY[ss->buff_w],
                    //     ss->speedAddZ[ss->buff_w],
                    //     ss->temp[ss->buff_w]);

                    if (ss->buff_w + 1 >= SERIAL_CIRCLE_BUFF_POINT)
                        ss->buff_w = 0;
                    else
                        ss->buff_w += 1;

                    pBuff += 70;
                    ret -= 70;
                }
                else
                {
                    pBuff += 1;
                    ret -= 1;
                }
            }
            //还有剩余
            if (ret > 0)
            {
                //移动内存
                for (count = 0; count < ret; count++)
                    buff[count] = pBuff[count];
            }
            //更新下一次的写入地址
            addr = ret;
        }
        else if (ret < 0)
            break;
    }
    close(ss->fd);
    ss->fd = 0;
    printf("serialSensor: thread exit \r\n");
}

/*
 *  返回0正常
 */
int serialSensor_get(float gyro[3], float accel[3], float compass[3], float angleAdd[3], float speedAdd[3], char *temp, uint32_t *timeStamp)
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

    if (serial_sensor->buff_r == serial_sensor->buff_w)
        return 1;

    if (accel)
    {
        accel[0] = serial_sensor->accX[serial_sensor->buff_r];
        accel[1] = serial_sensor->accY[serial_sensor->buff_r];
        accel[2] = serial_sensor->accZ[serial_sensor->buff_r];
    }
    if (gyro)
    {
        gyro[0] = serial_sensor->gyrX[serial_sensor->buff_r];
        gyro[1] = serial_sensor->gyrY[serial_sensor->buff_r];
        gyro[2] = serial_sensor->gyrZ[serial_sensor->buff_r];
    }
    if (compass)
    {
        compass[0] = serial_sensor->compassX[serial_sensor->buff_r];
        compass[1] = serial_sensor->compassY[serial_sensor->buff_r];
        compass[2] = serial_sensor->compassZ[serial_sensor->buff_r];
    }
    if (angleAdd)
    {
        angleAdd[0] = serial_sensor->angleAddX[serial_sensor->buff_r];
        angleAdd[1] = serial_sensor->angleAddY[serial_sensor->buff_r];
        angleAdd[2] = serial_sensor->angleAddZ[serial_sensor->buff_r];
    }
    if (speedAdd)
    {
        speedAdd[0] = serial_sensor->speedAddX[serial_sensor->buff_r];
        speedAdd[1] = serial_sensor->speedAddY[serial_sensor->buff_r];
        speedAdd[2] = serial_sensor->speedAddZ[serial_sensor->buff_r];
    }
    if (temp)
        *temp = serial_sensor->temp[serial_sensor->buff_r];
    if (timeStamp)
        *timeStamp = serial_sensor->timeStamp[serial_sensor->buff_r];

    if (serial_sensor->buff_r + 1 >= SERIAL_CIRCLE_BUFF_POINT)
        serial_sensor->buff_r = 0;
    else
        serial_sensor->buff_r += 1;

    return 0;
}
