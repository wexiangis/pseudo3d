/*
 *  根据MPU6050数据计算姿态
 */
#include <pthread.h>
#include "delayus.h"

#define POSTURE_WORK_MODE 1 // 0/原始数据模式 1/使用dmp库

#if(POSTURE_WORK_MODE == 1)
#include "inv_mpu.h"
#else
#include "mpu6050.h"
#define POSTURE_PI 3.14159265358979323846 //位数越多 精度越高
#endif

typedef struct
{
    pthread_t th;
    int flagRun;
    int intervalMs;
    //角速度、中立加速度除去的倍数
    float agPowReduce;
    float acPowReduce;
    //角速度原始数据
    short agXVal, agYVal, agZVal;
    //重力加速度原始数据
    short acXVal, acYVal, acZVal;
    //角速度累加得到的角度值(rad:0.0~2pi)
    float agX, agY, agZ;
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

void *posture_thread(void *argv)
{
#if(POSTURE_WORK_MODE == 1)
    mpu_dmp_init();
    while (ps.flagRun)
    {
        delayms(ps.intervalMs);
        mpu_dmp_get_data(
            &ps.agX, &ps.agY, &ps.agZ,
            &ps.agXVal, &ps.agYVal, &ps.agZVal,
            &ps.acXVal, &ps.acYVal, &ps.acZVal);
    }
#else
    //2倍pi值
    float val2p = POSTURE_PI * 2;
    mpu6050_init("/dev/i2c-1");
    while (ps.flagRun)
    {
        delayms(ps.intervalMs);
        //取角速度原始数据
        ps.agXVal = mpu6050_getGyro(0);
        ps.agYVal = mpu6050_getGyro(1);
        ps.agZVal = mpu6050_getGyro(2);
        //取重力加速度原始数据
        ps.acXVal = mpu6050_getAccel(0);
        ps.acYVal = mpu6050_getAccel(1);
        ps.acZVal = mpu6050_getAccel(2);
        //倍数转换 累加角度值
        ps.agX += (float)(ps.agXVal) / ps.agPowReduce / POSTURE_PI;
        ps.agY += (float)(ps.agYVal) / ps.agPowReduce / POSTURE_PI;
        ps.agZ += (float)(ps.agZVal) / ps.agPowReduce / POSTURE_PI;
        //
        if(ps.agX > POSTURE_PI) ps.agX -= val2p;
        else if(ps.agX < -POSTURE_PI) ps.agX += val2p;
        if(ps.agY > POSTURE_PI) ps.agY -= val2p;
        else if(ps.agY < -POSTURE_PI) ps.agY += val2p;
        if(ps.agZ > POSTURE_PI) ps.agZ -= val2p;
        else if(ps.agZ < -POSTURE_PI) ps.agZ += val2p;
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

//读取物体自身坐标系下的4个点(0,0,0),(1,0,0),(0,1,0),(0,0,1)在大地坐标系下的坐标
void posture_get(int *xyzo)
{
    ;
}

//获取转角
float posture_getX(void)
{
    return ps.agX;
}
float posture_getY(void)
{
    return ps.agY;
}
float posture_getZ(void)
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
