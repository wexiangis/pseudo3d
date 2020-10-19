/*
 *  根据MPU6050数据计算姿态
 */
#ifndef _POSTURE_H_
#define _POSTURE_H_

/*
 *  初始化
 * 
 *  intervalMs: 采样间隔, 越小误差积累越小, 建议值:10(推荐),20,25,50
 */
void posture_init(unsigned short intervalMs);
void posture_exit(void);

//复位(重置计算值)
void posture_reset(void);

//获取角速度计算的转角(相对于自身坐标系,rad:[-pi, pi])
float posture_getGX(void);
float posture_getGY(void);
float posture_getGZ(void);

//获取重力加速度计算的转角(相对于空间坐标系,rad:[-pi, pi])
float posture_getAX(void);
float posture_getAY(void);
float posture_getAZ(void);

//最终输出转角(相对于空间坐标系,rad:[-pi, pi])
float posture_getX(void);
float posture_getY(void);
float posture_getZ(void);

//获取加速度计数据
short posture_getAXVal(void);
short posture_getAYVal(void);
short posture_getAZVal(void);

//获取轴向加速度g值
float posture_getAXG(void);
float posture_getAYG(void);
float posture_getAZG(void);
float posture_getAG(void);

//获取陀螺仪数据
short posture_getGXVal(void);
short posture_getGYVal(void);
short posture_getGZVal(void);

//获取绕轴角速度rad/s
float posture_getGXR(void);
float posture_getGYR(void);
float posture_getGZR(void);

//获取罗盘角度(rad:[-pi, pi])
float posture_dir(void);

#endif
