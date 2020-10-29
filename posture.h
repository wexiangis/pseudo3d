/*
 *  根据MPU6050数据计算姿态
 */
#ifndef _POSTURE_H_
#define _POSTURE_H_

#include <pthread.h>

typedef struct
{
    //线程及其运行标志
    pthread_t th;
    short flagRun;
    //采样周期
    unsigned short intervalMs;
    //角速度原始数据
    short vGX, vGY, vGZ;
    //重力加速度原始数据
    short vAX, vAY, vAZ;
    //罗盘原始数据
    short vCX, vCY, vCZ;
    //水平方向(单位:rad)
    double dir;
    //温度(原始数值)
    long temper;
    //角速度累加得到的角度值(相对自身坐标,rad:[-pi, pi])
    double rGX, rGY, rGZ;
    //重力加速度得到的角度值(相对空间坐标,rad:[-pi, pi])
    double rAX, rAY, rAZ;
    //最终输出角度值(相对空间坐标,rad:[-pi, pi])
    double rX, rY, rZ;
    //偏航角较正
    double rZErr;
    //绕轴角速度(单位:rad/s)
    double vGX2, vGY2, vGZ2;
    //各轴受力及合力(单位:g)
    double vAX2, vAY2, vAZ2;
    //空间坐标系下的横纵向g值(单位:g)
    double gX, gY, gXYZ;
    //correct gX/gY (单位:g)
    double gXErr, gYErr;
    //accel in horizomtal X/Y (unit:m/ss)
    double aX, aY;
    //空间坐标系下的横纵向速度(单位:m/s)
    double speX, speY;
    //空间坐标系下的横纵向偏移距离(单位:m)
    double movX, movY;
    //
    int tt[4];
} PostureStruct;

/*
 *  初始化
 * 
 *  intervalMs: 采样间隔, 越小误差积累越小, 建议值:10(推荐),20,25,50
 */
PostureStruct *pe_init(unsigned short intervalMs);
void pe_exit(PostureStruct **ps);

//复位(重置计算值)
void pe_reset(PostureStruct *ps);

//获取罗盘角度(rad:[-pi, pi])
double pe_dir(void);

#endif
