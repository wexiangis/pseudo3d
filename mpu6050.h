#ifndef _MPU6050_H_
#define _MPU6050_H_

void mpu6050_init(char *i2cPath);
void mpu6050_release();

//获取加速度计数据 xyz = 0、1、2 分别对应x、y、z
short getAccel(unsigned char xyz);

//获取陀螺仪数据 xyz = 0、1、2 分别对应x、y、z
short getGyro(unsigned char xyz);

#endif
