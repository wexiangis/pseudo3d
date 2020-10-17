/*
 *  根据MPU6050数据计算姿态
 */
#ifndef _POSTURE_H_
#define _POSTURE_H_

//初始化,设定数据刷新间隔
void posture_init(int intervalMs);
void posture_exit(void);

//复位(重置计算值)
void posture_reset(void);

//获取角速度计算的转角(相对于自身坐标系)
double posture_getAGX(void);
double posture_getAGY(void);
double posture_getAGZ(void);

//获取重力加速度计算的转角(相对于空间坐标系)
double posture_getACX(void);
double posture_getACY(void);
double posture_getACZ(void);

//最终输出转角
double posture_getX(void);
double posture_getY(void);
double posture_getZ(void);

//获取加速度计数据
int posture_getAccelX(void);
int posture_getAccelY(void);
int posture_getAccelZ(void);

//获取陀螺仪数据
int posture_getGyroX(void);
int posture_getGyroY(void);
int posture_getGyroZ(void);

#endif
