/*
 *  接收来自串口的传感器数据
 */
#ifndef _SERIALSENSOR_H_
#define _SERIALSENSOR_H_

/*
 *  返回0正常
 */
int serialSensor_get(float *pry, float *gyro, float *accel);

#endif