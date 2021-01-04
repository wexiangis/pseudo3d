/*
 *  接收来自串口的传感器数据
 */
#ifndef _SERIALSENSOR_H_
#define _SERIALSENSOR_H_

#include <stdint.h>

#define SERIAL_DEV "/dev/ttyUSB0"
#define SERIAL_BAUND 460800
#define SERIAL_CIRCLE_BUFF_POINT 200 //缓存200个点(10ms间隔则2秒数据量)

/*
 *  返回0正常
 */
int serialSensor_get(float gyro[3], float accel[3], float compass[3], float angleAdd[3], float speedAdd[3], char *temp, uint32_t *timeStamp);

#endif