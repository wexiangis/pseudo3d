/*
 *  根据MPU6050数据计算姿态
 */
#include <math.h>
#include <pthread.h>
#include "delayus.h"

#define POSTURE_WORK_MODE 0 // 0/原始数据模式 1/使用dmp库

#if(POSTURE_WORK_MODE == 1)
#include "inv_mpu.h"
#else
#include "mpu6050.h"
#define PE_PI 3.14159265358979323846 //位数越多 精度越高
#define PE_2PI (PE_PI * 2)
#endif

typedef struct
{
    pthread_t th;
    int flagRun;
    int intervalMs;
    //角速度、中立加速度除去的倍数
    double agPowReduce;
    double acPowReduce;
    //角速度原始数据
    int agXVal, agYVal, agZVal;
    //重力加速度原始数据
    int acXVal, acYVal, acZVal;
    //角速度累加得到的角度值(rad:0.0~2pi)
    double agX, agY, agZ;
    //重力加速度得到的角度值(rad:0.0~2pi)
    double acX, acY, acZ;
} PostureStruct;

static PostureStruct ps = {
    .flagRun = 0,
    .intervalMs = 50,
    //角速度、中立加速度除去的倍数
    .agPowReduce = 5500, //relate to intervalMs
    .acPowReduce = 1,
    //角速度原始数据
    .agXVal = 0,
    .agYVal = 0,
    .agZVal = 0,
    //重力加速度原始数据
    .acXVal = 0,
    .acYVal = 0,
    .acZVal = 0,
    //角速度累加得到的角度值
    .agX = 0,
    .agY = 0,
    .agZ = 0,
    //重力加速度得到的角度值
    .acX = 0,
    .acY = 0,
    .acZ = 0,
};

int absInt16(int v)
{
    if (v < 0)
        return -v;
    return v;
}

void *posture_thread(void *argv)
{
#if(POSTURE_WORK_MODE == 1)
    double fVal[3];
    int sVal[6];
    mpu_dmp_init();
    while (ps.flagRun)
    {
        delayms(ps.intervalMs);
        if(mpu_dmp_get_data(fVal, sVal, &sVal[3]) == 0) {
            ps.agX = fVal[0];
            ps.agY = fVal[1];
            ps.agZ = fVal[2];
            ps.agXVal = sVal[0];
            ps.agYVal = sVal[1];
            ps.agZVal = sVal[2];
            ps.acXVal = sVal[3];
            ps.acYVal = sVal[4];
            ps.acZVal = sVal[5];
        }
    }
#else
    double tmp;
    //2倍pi值
    mpu6050_init("/dev/i2c-1");
    while (ps.flagRun)
    {
        delayms(ps.intervalMs);

        // ----- 采样 -----

        //取角速度原始数据
        ps.agXVal = mpu6050_getGyro(0);
        ps.agYVal = mpu6050_getGyro(1);
        ps.agZVal = mpu6050_getGyro(2);
        //取重力加速度原始数据
        ps.acXVal = mpu6050_getAccel(0);
        ps.acYVal = mpu6050_getAccel(1);
        ps.acZVal = mpu6050_getAccel(2);

        // ----- 角速度计算姿态 -----

        //倍数转换 累加角度值
        ps.agX += (double)(ps.agXVal) / ps.agPowReduce / PE_PI;
        ps.agY += (double)(ps.agYVal) / ps.agPowReduce / PE_PI;
        ps.agZ += (double)(ps.agZVal) / ps.agPowReduce / PE_PI;
        //范围限制
        if(ps.agX > PE_PI) ps.agX -= PE_2PI;
        else if(ps.agX < -PE_PI) ps.agX += PE_2PI;
        if(ps.agY > PE_PI) ps.agY -= PE_2PI;
        else if(ps.agY < -PE_PI) ps.agY += PE_2PI;
        if(ps.agZ > PE_PI) ps.agZ -= PE_2PI;
        else if(ps.agZ < -PE_PI) ps.agZ += PE_2PI;

        // ----- 重力加速度计算姿态 -----

        ps.acX = atan2((double)ps.acYVal, (double)ps.acZVal);
        tmp = sqrt((double)ps.acYVal * ps.acYVal + (double)ps.acZVal * ps.acZVal);
        ps.acY = -atan2((double)ps.acXVal, tmp);
        ps.acZ = 0;

        //ps.agX = ps.acX;
        //ps.acZ = ps.agZ;
    }
    mpu6050_release();
#endif

    return NULL;
}

//初始化,设定数据刷新间隔
void posture_init(int intervalMs)
{
    ps.intervalMs = intervalMs;
    if (!ps.flagRun)
        pthread_create(&ps.th, NULL, &posture_thread, NULL);
    ps.flagRun = 1;
}

void posture_exit(void)
{
    if(ps.flagRun)
    {
        ps.flagRun = 0;
        pthread_join(ps.th, NULL);
    }
}

//获取角速度计算的转角(相对于空间坐标系)
double posture_getAGX(void)
{
    return ps.agX;
}
double posture_getAGY(void)
{
    return ps.agY;
}
double posture_getAGZ(void)
{
    return ps.agZ;
}
//获取重力加速度计算的转角(相对于空间坐标系)
double posture_getACX(void)
{
    return ps.acX;
}
double posture_getACY(void)
{
    return ps.acY;
}
double posture_getACZ(void)
{
    return ps.acZ;
}
//最终输出转角
double posture_getX(void)
{
    return ps.agX * 0.5 + ps.acX * 0.5;
}
double posture_getY(void)
{
    return ps.agY * 0.5 + ps.acY * 0.5;
}
double posture_getZ(void)
{
    return ps.agZ * 0.5 + ps.acZ * 0.5;
}

//复位(重置计算值)
void posture_reset(void)
{
    ps.agX = ps.agY = ps.agZ = 0;
    ps.acX = ps.acY = ps.acZ = 0;
}

//获取加速度计数据
int posture_getAccelX(void)
{
    return ps.acXVal;
}
int posture_getAccelY(void)
{
    return ps.acYVal;
}
int posture_getAccelZ(void)
{
    return ps.acZVal;
}

//获取陀螺仪数据
int posture_getGyroX(void)
{
    return ps.agXVal;
}
int posture_getGyroY(void)
{
    return ps.agYVal;
}
int posture_getGyroZ(void)
{
    return ps.agZVal;
}
