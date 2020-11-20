#ifndef _MPU6050_H_
#define _MPU6050_H_

int mpu6050_init(unsigned short Hz, char test);

int mpu6050_angle(float *pry, float *gyro, float *accel);

int mpu6050_compass(float *compass);

int mpu6050_temper(float *temper);

#endif
