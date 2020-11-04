#ifndef _MPU6050_H_
#define _MPU6050_H_

//1弧度对应采样值,陀螺仪数据除以该值得到绕轴角加速度,单位rad/s
#define GYRO_VAL_P_RED (32768 / 2000 * PE_PI / 180)

//1g对应采样值,加速度计数据除以该值得到轴向受力,单位g
#define ACCEL_VAL_P_G (32768 / 2)

//1uT对应采样值,罗盘数据除以该值得到轴向受力,单位uT (纠正中...)
#define COMPASS_VAL_P_G (8192 / 4800)

//1度对应采样值,温度据除以该值得到轴向受力,单位度 (纠正中...)
#define TEMPER_VAL_P_G (32768)

int mpu6050_init(unsigned short Hz, char test);

int mpu6050_angle(double *pry, short *gyro, short *accel);

int mpu6050_compass(short *compass);

int mpu6050_temper(long *temper);

#endif
