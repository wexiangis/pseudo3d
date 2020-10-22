#ifndef _MPU6050_H_
#define _MPU6050_H_

int mpu6050_init(unsigned short Hz, char test);
int mpu6050_get(double *pry, short *gyro, short *accel, short *dir);

#endif