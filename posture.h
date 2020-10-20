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
void pe_init(unsigned short intervalMs);
void pe_exit(void);

//复位(重置计算值)
void pe_reset(void);

//获取角速度计算的转角(相对于自身坐标系,rad:[-pi, pi])
float pe_getGX(void);
float pe_getGY(void);
float pe_getGZ(void);

//获取重力加速度计算的转角(相对于空间坐标系,rad:[-pi, pi])
float pe_getAX(void);
float pe_getAY(void);
float pe_getAZ(void);

//最终输出转角(相对于空间坐标系,rad:[-pi, pi])
float pe_getX(void);
float pe_getY(void);
float pe_getZ(void);

//获取加速度计数据
short pe_getAXVal(void);
short pe_getAYVal(void);
short pe_getAZVal(void);

//获取轴向加速度g值
float pe_getAXG(void);
float pe_getAYG(void);
float pe_getAZG(void);
float pe_getAG(void);

//获取陀螺仪数据
short pe_getGXVal(void);
short pe_getGYVal(void);
short pe_getGZVal(void);

//获取绕轴角速度rad/s
float pe_getGXR(void);
float pe_getGYR(void);
float pe_getGZR(void);

//获取罗盘角度(rad:[-pi, pi])
float pe_dir(void);

#endif
