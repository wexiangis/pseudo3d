/*
 *  根据MPU6050数据计算姿态
 */
#include <math.h>
#include <pthread.h>
#include "delayus.h"

// 0/原始数据模式 1/使用dmp库 2/compare
#define PE_WORK_MODE 2

// 启用罗盘
#define PE_USE_HMC5883 0
#if (PE_USE_HMC5883 != 0)
#include "hmc5883.h"
#endif

#if (PE_WORK_MODE > 0)
#include "inv_mpu.h"
#else
#include "mpu6050.h"
#endif

#define PE_PI 3.14159265358979323846 //位数越多 精度越高
#define PE_2PI (PE_PI * 2)

// param in 100Hz
#define PE_ACP (PE_PI * 3000)

typedef struct
{
    //线程及其运行标志
    pthread_t th;
    short flagRun;
    //采样周期
    unsigned short intervalMs;
    //角速度原始数据
    short agXVal, agYVal, agZVal;
    //重力加速度原始数据
    short acXVal, acYVal, acZVal;
    //角速度累加得到的角度值(相对自身坐标,rad:[-pi, pi])
    float agX, agY, agZ;
    //重力加速度得到的角度值(相对空间坐标,rad:[-pi, pi])
    float acX, acY, acZ;
    //最终输出角度值(相对空间坐标,rad:[-pi, pi])
    float X, Y, Z;
    //偏航角较正
    float zErr;
} PostureStruct;

static PostureStruct ps = {
    .flagRun = 0,
    .intervalMs = 50,
    //角速度原始数据
    .agXVal = 0,
    .agYVal = 0,
    .agZVal = 0,
    //重力加速度原始数据
    .acXVal = 0,
    .acYVal = 0,
    .acZVal = 0,
    //角速度累加得到的角度值(相对自身坐标)
    .agX = 0,
    .agY = 0,
    .agZ = 0,
    //重力加速度得到的角度值(相对空间坐标)
    .acX = 0,
    .acY = 0,
    .acZ = 0,
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
    short agVal[3];
    short acVal[3];
#if (PE_WORK_MODE == 1)
    float fVal[3];
    mpu_dmp_init(1000 / ps.intervalMs, 0);
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
        ps.agXVal = agVal[0];
        ps.agYVal = agVal[1];
        ps.agZVal = agVal[2];
        ps.acXVal = acVal[0];
        ps.acYVal = acVal[1];
        ps.acZVal = acVal[2];
    }
#else
    float tmp;
#if (PE_WORK_MODE == 2)
    float fVal[3];
    mpu_dmp_init(1000 / ps.intervalMs, 0);
#else
    //2倍pi值
    mpu6050_init("/dev/i2c-1");
#endif
    while (ps.flagRun)
    {
        DELAY_US(ps.intervalMs * 1000);

        // ----- 采样 -----
#if (PE_WORK_MODE == 2)
        if (mpu_dmp_get_data(fVal, agVal, acVal) == 0)
        {
            ps.X = fVal[1] * PE_PI / 180;
            ps.Y = fVal[0] * PE_PI / 180;
            ps.Z = fVal[2] * PE_PI / 180;
        }
#else
        //取角速度原始数据
        agVal[0] = mpu6050_getGyro(0);
        agVal[1] = mpu6050_getGyro(1);
        agVal[2] = mpu6050_getGyro(2);
        //取重力加速度原始数据
        acVal[0] = mpu6050_getAccel(0);
        acVal[1] = mpu6050_getAccel(1);
        acVal[2] = mpu6050_getAccel(2);
#endif
        ps.agXVal = agVal[0];
        ps.agYVal = agVal[1];
        ps.agZVal = agVal[2];
        ps.acXVal = acVal[0];
        ps.acYVal = acVal[1];
        ps.acZVal = acVal[2];

        // ----- ag计算姿态 -----

        //累加角度值
        ps.agX += (float)agVal[0] / ps.intervalMs / PE_ACP;
        ps.agY += (float)agVal[1] / ps.intervalMs / PE_ACP;
        ps.agZ += (float)agVal[2] / ps.intervalMs / PE_ACP;
        //范围限制
        if (ps.agX > PE_PI)
            ps.agX -= PE_2PI;
        else if (ps.agX < -PE_PI)
            ps.agX += PE_2PI;
        if (ps.agY > PE_PI)
            ps.agY -= PE_2PI;
        else if (ps.agY < -PE_PI)
            ps.agY += PE_2PI;
        if (ps.agZ > PE_PI)
            ps.agZ -= PE_2PI;
        else if (ps.agZ < -PE_PI)
            ps.agZ += PE_2PI;

        // ----- accel计算姿态 -----

        ps.acX = atan2((float)ps.acYVal, (float)ps.acZVal);
        tmp = (float)sqrt((float)ps.acYVal * ps.acYVal + (float)ps.acZVal * ps.acZVal);
        ps.acY = -atan2((float)ps.acXVal, tmp);
        ps.acZ = 0;
    }
#if (PE_WORK_MODE == 0)
    mpu6050_release();
#endif
#endif

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
float posture_getAGX(void)
{
    return ps.agX;
}
float posture_getAGY(void)
{
    return ps.agY;
}
float posture_getAGZ(void)
{
    return ps.agZ;
}
//获取重力加速度计算的转角(相对于空间坐标系,rad:[-pi, pi])
float posture_getACX(void)
{
    return ps.acX;
}
float posture_getACY(void)
{
    return ps.acY;
}
float posture_getACZ(void)
{
    return ps.acZ;
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
    ps.agX = ps.agY = ps.agZ = 0;
    ps.acX = ps.acY = ps.acZ = 0;
#if (PE_WORK_MODE > 0)
    ps.zErr = -ps.Z;
#endif
}

//获取加速度计数据
short posture_getACXVal(void)
{
    return ps.acXVal;
}
short posture_getACYVal(void)
{
    return ps.acYVal;
}
short posture_getACZVal(void)
{
    return ps.acZVal;
}

//获取陀螺仪数据
short posture_getAGXVal(void)
{
    return ps.agXVal;
}
short posture_getAGYVal(void)
{
    return ps.agYVal;
}
short posture_getAGZVal(void)
{
    return ps.agZVal;
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
