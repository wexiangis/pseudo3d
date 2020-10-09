/*
 *  MPU6050 I2C 驱动
 */
#ifndef _MPU6050_H_
#define _MPU6050_H_

void mpu6050_init(char *i2cPath);
void mpu6050_release();

//获取加速度计数据 xyz = 0、1、2 分别对应x、y、z
short mpu6050_getAccel(unsigned char xyz);

//获取陀螺仪数据 xyz = 0、1、2 分别对应x、y、z
short mpu6050_getGyro(unsigned char xyz);

#endif
