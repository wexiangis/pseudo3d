#include <stdio.h>

#define SMPLRT_DIV 0x19   //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIG 0x1A       //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG 0x1B  //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define ACCEL_CONFIG 0x1C //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define PWR_MGMT_1 0x6B //电源管理，典型值：0x00(正常启用)
#define WHO_AM_I 0x75   //IIC地址寄存器(默认数值0x68，只读)
#define SLAVE_ADDR 0xD0 //IIC写入时的地址字节数据，+1为读取

#define GX_Offset -65
#define GY_Offset 14
#define GZ_Offset -23

/*
 *  i2c operation
 */
#include "i2c_transfer.h"

static int _fd = 0;

static void mpu6050_write(unsigned char reg, unsigned char val)
{
    unsigned char data_buf[1] = {0};
    unsigned char reg_addr[1] = {0};
    if (_fd < 1)
        return;
    data_buf[0] = val;
    reg_addr[0] = reg;
    i2c_transfer_write(_fd, SLAVE_ADDR, data_buf, 1, reg_addr, 1);
}

static unsigned char mpu6050_read(unsigned char reg)
{
    unsigned char data_buf[1] = {0};
    unsigned char reg_addr[1] = {0};
    if (_fd < 1)
        return 0x00;
    reg_addr[0] = reg;
    if (i2c_transfer_read(_fd, SLAVE_ADDR, data_buf, 1, reg_addr, 1) == 0)
        return data_buf[0];
    return 0x00;
}

/*
 *  mpu6050 operation
 */
void mpu6050_release()
{
    if (_fd > 0)
        i2c_transfer_close(_fd);
    _fd = 0;
}

void mpu6050_init(char *i2cPath)
{
    mpu6050_release();

    _fd = i2c_transfer_open(i2cPath);
    if (_fd < 1)
    {
        fprintf(stderr, "mpu6050_init: open %s err\r\n", i2cPath);
        return;
    }

    mpu6050_write(PWR_MGMT_1, 0x00); //解除休眠状态
    mpu6050_write(SMPLRT_DIV, 0x07);
    mpu6050_write(CONFIG, 0x06);
    mpu6050_write(GYRO_CONFIG, 0x18);
    mpu6050_write(ACCEL_CONFIG, 0x01);
}

short mpu6050_getData(unsigned char reg)
{
    short H = 0, L = 0;
    H = mpu6050_read(reg);
    L = mpu6050_read(reg + 1);
    return (H << 8) + L;
}

//获取加速度计数据 xyz = 0、1、2 分别对应x、y、z
short getAccel(unsigned char xyz)
{
    if (xyz == 0)
        return mpu6050_getData(ACCEL_XOUT_H);
    else if (xyz == 1)
        return mpu6050_getData(ACCEL_YOUT_H);
    else
        return mpu6050_getData(ACCEL_ZOUT_H);
}

//获取陀螺仪数据 xyz = 0、1、2 分别对应x、y、z
short getGyro(unsigned char xyz)
{
    if (xyz == 0)
        return mpu6050_getData(GYRO_XOUT_H) - GX_Offset;
    else if (xyz == 1)
        return mpu6050_getData(GYRO_YOUT_H) - GY_Offset;
    else
        return mpu6050_getData(GYRO_ZOUT_H) - GZ_Offset;
}
