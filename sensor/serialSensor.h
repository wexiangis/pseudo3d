/*
 *  接收来自串口的传感器数据
 */
#ifndef _SERIALSENSOR_H_
#define _SERIALSENSOR_H_

#define SERIAL_DEV "/dev/ttyUSB0"
#define SERIAL_BAUND 460800
#define SERIAL_INTERVALMS 2
#define SERIAL_CIRCLE_BUFF_POINT 500 //缓存500个点(10ms间隔则5秒数据量)

/*
 *  返回0正常
 */
int serialSensor_get(float gyro[3], float accel[3]);

#endif