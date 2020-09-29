/*
 *  根据MPU6050数据计算姿态
 */
#include <pthread.h>
#include "mpu6050.h"

#define POSTURE_PI 3.14159265358979323846 //位数越多 精度越高

typedef struct
{
    pthread_t th;
    int flagRun;
    int intervalMs;
    //角速度、中立加速度除去的倍数
    double agPowReduce;
    double acPowReduce;
    //角速度原始数据
    short agXVal, agYVal, agZVal;
    //重力加速度原始数据
    short acXVal, acYVal, acZVal;
    //角速度累加得到的角度值
    double agX, agY, agZ;
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
};

/* 稍微精准的延时 */
#include <sys/time.h>
static void delayms(unsigned int ms)
{
    struct timeval delay;
    delay.tv_sec = ms / 1000;
    delay.tv_usec = ms % 1000 * 1000;
    select(0, NULL, NULL, NULL, &delay);
}

void *posture_thread(void *argv)
{
    double val2p = POSTURE_PI * 2;
    mpu6050_init("/dev/i2c-1");
    while (ps.flagRun)
    {
        delayms(ps.intervalMs);
        //取角速度 倍数转换
        ps.agXVal = mpu6050_getGyro(0);
        ps.agYVal = mpu6050_getGyro(1);
        ps.agZVal = mpu6050_getGyro(2);
        //取重力加速度
        ps.acXVal = mpu6050_getAccel(0);
        ps.acYVal = mpu6050_getAccel(1);
        ps.acZVal = mpu6050_getAccel(2);
        //累加角度值
        ps.agX += (double)(ps.agXVal) / ps.agPowReduce / POSTURE_PI;
        ps.agY += (double)(ps.agYVal) / ps.agPowReduce / POSTURE_PI;
        ps.agZ += (double)(ps.agZVal) / ps.agPowReduce / POSTURE_PI;
        //
        if(ps.agX > POSTURE_PI) ps.agX -= val2p;
        else if(ps.agX < -POSTURE_PI) ps.agX += val2p;
        if(ps.agY > POSTURE_PI) ps.agY -= val2p;
        else if(ps.agY < -POSTURE_PI) ps.agY += val2p;
        if(ps.agZ > POSTURE_PI) ps.agZ -= val2p;
        else if(ps.agZ < -POSTURE_PI) ps.agZ += val2p;
    }
    mpu6050_release();
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

//读取物体自身坐标系下的4个点(0,0,0),(1,0,0),(0,1,0),(0,0,1)在大地坐标系下的坐标
void posture_get(int *xyzo)
{
    ;
}

//获取转角
double posture_getX(void)
{
    return ps.agX;
}
double posture_getY(void)
{
    return ps.agY;
}
double posture_getZ(void)
{
    return ps.agZ;
}

//复位
void posture_reset(void)
{
    ps.agX = ps.agY = ps.agZ = 0;
}

//获取加速度计数据
short posture_getAccelX(void)
{
    return ps.acXVal;
}
short posture_getAccelY(void)
{
    return ps.acYVal;
}
short posture_getAccelZ(void)
{
    return ps.acZVal;
}

//获取陀螺仪数据
short posture_getGyroX(void)
{
    return ps.agXVal;
}
short posture_getGyroY(void)
{
    return ps.agYVal;
}
short posture_getGyroZ(void)
{
    return ps.agZVal;
}
