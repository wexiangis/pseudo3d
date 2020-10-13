/*
 *  根据MPU6050数据计算姿态
 */
#ifndef _POSTURE_H_
#define _POSTURE_H_

//初始化,设定数据刷新间隔
void posture_init(int intervalMs);
void posture_exit(void);

//读取物体自身坐标系下的4个点(0,0,0),(1,0,0),(0,1,0),(0,0,1)在大地坐标系下的坐标
void posture_get(int *xyzo);

//复位
void posture_reset(void);

//获取角速度计算的转角
float posture_getAGX(void);
float posture_getAGY(void);
float posture_getAGZ(void);
//获取重力加速度计算的转角
float posture_getACX(void);
float posture_getACY(void);
float posture_getACZ(void);
//最终输出转角
float posture_getX(void);
float posture_getY(void);
float posture_getZ(void);

//获取加速度计数据
short posture_getAccelX(void);
short posture_getAccelY(void);
short posture_getAccelZ(void);

//获取陀螺仪数据
short posture_getGyroX(void);
short posture_getGyroY(void);
short posture_getGyroZ(void);

#endif
