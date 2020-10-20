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
    //角速度累加得到的角度值(相对自身坐标,rad:[-pi, pi])
    float gX, gY, gZ;
    //重力加速度得到的角度值(相对空间坐标,rad:[-pi, pi])
    float aX, aY, aZ;
    //最终输出角度值(相对空间坐标,rad:[-pi, pi])
    float rX, rY, rZ;
    //偏航角较正
    float zErr;
    //
    float gXR, gYR, gZR;
    //
    float aXG, aYG, aZG, aG;
    //
    float xG, yG;
    //
    float xSpe, ySpe;
    //
    float xMov, yMov;
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
float pe_dir(void);

#endif
