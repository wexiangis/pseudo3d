/*
 *  接收来自串口的传感器数据
 */
#ifndef _SERIALSENSOR_H_
#define _SERIALSENSOR_H_

#define SERIAL_DEV "/dev/ttyUSB0"
#define SERIAL_BAUND 460800
#define SERIAL_INTERVALMS 2
#define SERIAL_CIRCLE_BUFF_LEN 102400

/*
 *  返回0正常
 */
int serialSensor_get(float *gyro, float *accel);

#endif