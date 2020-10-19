/*
 *  根据MPU6050数据计算姿态
 */
#include <math.h>
#include <pthread.h>

#include "delayus.h"
#include "inv_mpu.h"

// 启用罗盘
#define PE_USE_HMC5883 0
#if (PE_USE_HMC5883 != 0)
#include "hmc5883.h"
#endif

#define PE_PI 3.14159265358979323846
#define PE_2PI (PE_PI * 2)

// 1弧度对应采样值, 陀螺仪数据除以该值得到绕轴角加速度, 单位rad/s
#define GYRO_VAL_P_RED (32768 / 2000 * PE_PI / 180)

// 1g对应采样值, 加速度计数据除以该值得到轴向受力, 单位g
#define ACCEL_VAL_P_G (32768 / 2)

// 陀螺仪积分时矫正倍数
#define GYRO_REDUCE_POW 1000

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
    float X, Y, Z;
    //偏航角较正
    float zErr;
} PostureStruct;

static PostureStruct ps = {
    .flagRun = 0,
    .intervalMs = 10,
    //角速度原始数据
    .gXVal = 0,
    .gYVal = 0,
    .gZVal = 0,
    //重力加速度原始数据
    .aXVal = 0,
    .aYVal = 0,
    .aZVal = 0,
    //角速度累加得到的角度值(相对自身坐标)
    .gX = 0,
    .gY = 0,
    .gZ = 0,
    //重力加速度得到的角度值(相对空间坐标)
    .aX = 0,
    .aY = 0,
    .aZ = 0,
    //最终输出角度值(相对空间坐标)
    .X = 0,
    .Y = 0,
    .Z = 0,
    //偏航角较正
    .zErr = 0,
};

void *posture_thread(void *argv)
{
    DELAY_US_INIT;
    float fVal[3];
    short agVal[3], acVal[3];
    //初始化mpu6050
    if (mpu_dmp_init(1000 / ps.intervalMs, 0) != 0)
        return NULL;
    //周期采样
    while (ps.flagRun)
    {
        DELAY_US(ps.intervalMs * 1000);

        // ----- 采样 -----
        if (mpu_dmp_get_data(fVal, agVal, acVal) == 0)
        {
            ps.X = fVal[1] * PE_PI / 180;
            ps.Y = fVal[0] * PE_PI / 180;
            ps.Z = fVal[2] * PE_PI / 180;
        }

        // 累加角加速度得到角度值
        ps.gX += ((float)agVal[0] - (float)(agVal[0] - ps.gXVal) / 2) * ps.intervalMs / 1000 / GYRO_REDUCE_POW;
        ps.gY += ((float)agVal[1] - (float)(agVal[1] - ps.gYVal) / 2) * ps.intervalMs / 1000 / GYRO_REDUCE_POW;
        ps.gZ += ((float)agVal[2] - (float)(agVal[2] - ps.gZVal) / 2) * ps.intervalMs / 1000 / GYRO_REDUCE_POW;

        // 范围限制
        if (ps.gX > PE_PI)
            ps.gX -= PE_2PI;
        else if (ps.gX < -PE_PI)
            ps.gX += PE_2PI;
        if (ps.gY > PE_PI)
            ps.gY -= PE_2PI;
        else if (ps.gY < -PE_PI)
            ps.gY += PE_2PI;
        if (ps.gZ > PE_PI)
            ps.gZ -= PE_2PI;
        else if (ps.gZ < -PE_PI)
            ps.gZ += PE_2PI;

        // copy
        ps.gXVal = agVal[0];
        ps.gYVal = agVal[1];
        ps.gZVal = agVal[2];
        ps.aXVal = acVal[0];
        ps.aYVal = acVal[1];
        ps.aZVal = acVal[2];

        // accel计算姿态
        ps.aX = atan2((float)ps.aYVal, (float)ps.aZVal);
        ps.aY = -atan2((float)ps.aXVal,
                (float)sqrt((float)ps.aYVal * ps.aYVal + (float)ps.aZVal * ps.aZVal));
        ps.aZ = 0;
    }

    return NULL;
}

/*
 *  初始化
 * 
 *  intervalMs: 采样间隔, 越小误差积累越小, 建议值:10(推荐),20,25,50
 */
void posture_init(unsigned short intervalMs)
{
    ps.intervalMs = intervalMs;
    if (!ps.flagRun)
        pthread_create(&ps.th, NULL, &posture_thread, NULL);
    ps.flagRun = 1;
}

void posture_exit(void)
{
    if (ps.flagRun)
    {
        ps.flagRun = 0;
        pthread_join(ps.th, NULL);
    }
}

//获取角速度计算的转角(相对于空间坐标系,rad:[-pi, pi])
float posture_getGX(void)
{
    return ps.gX;
}
float posture_getGY(void)
{
    return ps.gY;
}
float posture_getGZ(void)
{
    return ps.gZ;
}
//获取重力加速度计算的转角(相对于空间坐标系,rad:[-pi, pi])
float posture_getAX(void)
{
    return ps.aX;
}
float posture_getAY(void)
{
    return ps.aY;
}
float posture_getAZ(void)
{
    return ps.aZ;
}
//最终输出转角(相对于空间坐标系,rad:[-pi, pi])
float posture_getX(void)
{
    return ps.X;
}
float posture_getY(void)
{
    return ps.Y;
}
float posture_getZ(void)
{
    return ps.Z + ps.zErr;
}

//复位(重置计算值)
void posture_reset(void)
{
    ps.gX = ps.gY = ps.gZ = 0;
    ps.aX = ps.aY = ps.aZ = 0;
    ps.zErr = -ps.Z;
}

//获取加速度计数据
short posture_getAXVal(void)
{
    return ps.aXVal;
}
short posture_getAYVal(void)
{
    return ps.aYVal;
}
short posture_getAZVal(void)
{
    return ps.aZVal;
}

//获取轴向加速度g值
float posture_getAXG(void)
{
    return (float)ps.aXVal / ACCEL_VAL_P_G;
}
float posture_getAYG(void)
{
    return (float)ps.aYVal / ACCEL_VAL_P_G;
}
float posture_getAZG(void)
{
    return (float)ps.aZVal / ACCEL_VAL_P_G;
}

//获取陀螺仪数据
short posture_getGXVal(void)
{
    return ps.gXVal;
}
short posture_getGYVal(void)
{
    return ps.gYVal;
}
short posture_getGZVal(void)
{
    return ps.gZVal;
}

//获取绕轴角速度rad/s
float posture_getGXR(void)
{
    return (float)ps.gXVal / GYRO_VAL_P_RED;
}
float posture_getGYR(void)
{
    return (float)ps.gYVal / GYRO_VAL_P_RED;
}
float posture_getGZR(void)
{
    return (float)ps.gZVal / GYRO_VAL_P_RED;
}

//获取罗盘角度(rad:[-pi, pi])
float posture_dir(void)
{
#if (PE_USE_HMC5883 > 0)
    return hmc5883_get();
#else
    return 0;
#endif
}
