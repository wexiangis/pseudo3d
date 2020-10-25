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
    short gXVal, gYVal, gZVal;
    //重力加速度原始数据
    short aXVal, aYVal, aZVal;
    //罗盘原始数据
    short cXVal, cYVal, cZVal;
    //水平方向(单位:rad)
    double dir;
    //温度(原始数值)
    long temper;
    //角速度累加得到的角度值(相对自身坐标,rad:[-pi, pi])
    double gX, gY, gZ;
    //重力加速度得到的角度值(相对空间坐标,rad:[-pi, pi])
    double aX, aY, aZ;
    //最终输出角度值(相对空间坐标,rad:[-pi, pi])
    double rX, rY, rZ;
    //偏航角较正
    double zErr;
    //绕轴角速度(单位:rad/s)
    double gXR, gYR, gZR;
    //各轴受力及合力(单位:g)
    double aXG, aYG, aZG, aG;
    //空间坐标系下的横纵向g值(单位:g)
    double xG, yG;
    //空间坐标系下的横纵向速度(单位:m/s)
    double xSpe, ySpe;
    //空间坐标系下的横纵向偏移距离(单位:m)
    double xMov, yMov;
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
