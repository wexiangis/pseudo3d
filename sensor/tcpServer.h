/*
 *  接收来自android设备的姿态参数
 */
#ifndef _TCPSERVER_H_
#define _TCPSERVER_H_

#define TCPSERVER_PORT_GYR 1201 //陀螺仪数据端口
#define TCPSERVER_PORT_ACC 1202 //加速度数据端口
#define TCPSERVER_PORT_PRY 1203 //欧拉角数据端口

/*
 *  返回0正常
 */
int tcpServer_get(float *pry, float *gyro, float *accel);

#endif