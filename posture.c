/*
 *  根据MPU6050数据计算姿态
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "posture.h"
#include "delayus.h"
#include "mpu6050.h"
#include "pseudo3d.h"
#include "dot.h"

// 启用HMC5883罗盘
#define PE_USE_HMC5883 0
#if (PE_USE_HMC5883 != 0)
#include "hmc5883.h"
#endif

#define PE_PI 3.14159265358979323846
#define PE_2PI (PE_PI * 2)

// (unit:N/kg)
#define PE_GRAVITY 9.8
// (unit:kg)
#define PE_MASS 1

// 陀螺仪积分矫正倍数
#define GYRO_REDUCE_POW 1000

// 陀螺仪角度积分公式(这里用的梯形面积计算方式)
// 采样间隔 intervalMs 变化时不需再调整参数
#define GYRO_SUM_FUN(new, old) \
((double)(new + old) / 2 * ps->intervalMs / 1000 / GYRO_REDUCE_POW)

// Speed积分矫正倍数
#define SPE_REDUCE_POW 1

// Speed积分得到移动距离(同上面 GYRO_SUM_FUN())
#define SPE_SUN_FUN(new, old) \
((new + old) / 2 * ps->intervalMs / 1000 / SPE_REDUCE_POW)

// Mov积分矫正倍数
#define MOV_REDUCE_POW 1

// Mov积分得到移动距离(同上面 GYRO_SUM_FUN())
#define MOV_SUN_FUN(new, old) \
((new + old) / 2 * ps->intervalMs / 1000 / MOV_REDUCE_POW)

void pe_inertial_navigation(PostureStruct *ps)
{
    double speX, speY, aX, aY;
    //
    aX = ps->gX * PE_GRAVITY / PE_MASS;
    aY = ps->gY * PE_GRAVITY / PE_MASS;
    //g值积分得到速度
    speX = ps->speX + SPE_SUN_FUN(aX, ps->aX);
    speY = ps->speY + SPE_SUN_FUN(aY, ps->aY);
    //速度积分得到移动距离
    ps->movX += MOV_SUN_FUN(speX, ps->speX);
    ps->movY += MOV_SUN_FUN(speY, ps->speY);
    //bakup
    ps->aX = aX;
    ps->aY = aY;
    ps->speX = speX;
    ps->speY = speY;
}

void pe_accel(PostureStruct *ps, short *valA)
{
    double err = 0.0001;
    double vXYZ[3], rXYZ[3];
    // bakup
    ps->vAX = valA[0];
    ps->vAY = valA[1];
    ps->vAZ = valA[2];
    // accel计算姿态
    ps->rAX = atan2((double)ps->vAY, (double)ps->vAZ);
    ps->rAY = -atan2((double)ps->vAX,
        (double)sqrt((double)ps->vAY * ps->vAY + (double)ps->vAZ * ps->vAZ));
    ps->rAZ = 0;
    // 各轴向受力转换(单位:g)
    ps->vAX2 = (double)ps->vAX / ACCEL_VAL_P_G;
    ps->vAY2 = (double)ps->vAY / ACCEL_VAL_P_G;
    ps->vAZ2 = (double)ps->vAZ / ACCEL_VAL_P_G;
    //
    vXYZ[0] = ps->vAX2; vXYZ[1] = ps->vAY2; vXYZ[2] = ps->vAZ2;
    rXYZ[0] = ps->rX; rXYZ[1] = ps->rY; rXYZ[2] = ps->rZ;
    //用逆矩阵把三轴受力的合向量转为空间坐标系下的向量
    p3d_matrix_zyx(rXYZ, vXYZ);
    //则该向量在水平方向的分量即为横纵向的g值
    ps->gX = vXYZ[0] + ps->gXErr;
    ps->gY = vXYZ[1] + ps->gYErr;
#if 1
    // adjust
    if (ps->gX > err) ps->gXErr -= err;
    else if (ps->gX < -err) ps->gXErr += err;
    if (ps->gY > err) ps->gYErr -= err;
    else if(ps->gY < -err) ps->gYErr += err;
#endif
    // 合受力(单位:g)
    ps->gXYZ = sqrt(ps->vAX2 * ps->vAX2 + ps->vAY2 * ps->vAY2 + ps->vAZ2 * ps->vAZ2);
    //
    dot_set(ps->gX, 0, 0xFF0000);
    dot_set(0, ps->gY, 0x0000FF);
    dot_set(ps->gX, ps->gY, 0xFFFF00);
}

void pe_gyro(PostureStruct *ps, short *valG)
{
    // 累加角加速度得到角度值
    ps->rGX += GYRO_SUM_FUN(valG[0], ps->vGX);
    ps->rGY += GYRO_SUM_FUN(valG[1], ps->vGY);
    ps->rGZ += GYRO_SUM_FUN(valG[2], ps->vGZ);
    // 范围限制
    if (ps->rGX > PE_PI)
        ps->rGX -= PE_2PI;
    else if (ps->rGX < -PE_PI)
        ps->rGX += PE_2PI;
    if (ps->rGY > PE_PI)
        ps->rGY -= PE_2PI;
    else if (ps->rGY < -PE_PI)
        ps->rGY += PE_2PI;
    if (ps->rGZ > PE_PI)
        ps->rGZ -= PE_2PI;
    else if (ps->rGZ < -PE_PI)
        ps->rGZ += PE_2PI;
    // copy
    ps->vGX = valG[0];
    ps->vGY = valG[1];
    ps->vGZ = valG[2];
    // 角速度转换(单位:rad/s)
    ps->vGX2 = (double)ps->vGX / GYRO_VAL_P_RED;
    ps->vGY2 = (double)ps->vGY / GYRO_VAL_P_RED;
    ps->vGZ2 = (double)ps->vGZ / GYRO_VAL_P_RED;
}

void *pe_thread(void *argv)
{
    DELAY_US_INIT;
    int timeCount = 0;
    double valR[3];
    short valG[3], valA[3], valC[3];
    PostureStruct *ps = (PostureStruct*)argv;
    //初始化mpu6050
    if (mpu6050_init(1000 / ps->intervalMs, 0) != 0)
        return NULL;
    //周期采样
    while (ps->flagRun)
    {
        DELAY_US(ps->intervalMs * 1000);
        // 采样
        if (mpu6050_angle(valR, valG, valA) == 0)
        {
            // dmp库用4元素法得到的欧拉角(单位:rad)
            ps->rX = valR[1];
            ps->rY = valR[0];
            ps->rZ = valR[2] + ps->rZErr;
            // adjust gyro direction
            valG[0] = -valG[0];
            valG[1] = -valG[1];
            valG[2] = -valG[2];
        }
        // gyro: 
        pe_gyro(ps, valG);
        // accel: 
        pe_accel(ps, valA);
        // 惯导参数计算
        pe_inertial_navigation(ps);
        //定时获取罗盘数据 & 温度数据
        timeCount += ps->intervalMs;
        if(timeCount >= 200){
            timeCount = 0;
            //罗盘
            if(mpu6050_compass(valC) == 0) {
                ps->vCX = valC[0];
                ps->vCY = valC[1];
                ps->vCZ = valC[2];
                ps->dir = atan2((float)ps->vCY, (float)ps->vCX);
            }
            //温度
            mpu6050_temper(&ps->temper);
        }
    }
    return NULL;
}

//复位(重置计算值)
void pe_reset(PostureStruct *ps)
{
    ps->rGX = ps->rGY = ps->rGZ = 0;
    ps->rAX = ps->rAY = ps->rAZ = 0;
    ps->rZErr -= ps->rZ;
    ps->aX = ps->aY = 0;
    ps->speX = ps->speY = 0;
    ps->movX = ps->movY = 0;
    dot_clear();
    ps->tt[0] = ps->tt[1] = ps->tt[2] = ps->tt[3] = 0;
}

/*
 *  初始化
 * 
 *  intervalMs: 采样间隔, 越小误差积累越小, 建议值: 2, 5, 10(推荐),20,25,50
 */
PostureStruct *pe_init(unsigned short intervalMs)
{
    PostureStruct *ps = (PostureStruct*)calloc(1, sizeof(PostureStruct));
    ps->intervalMs = intervalMs;
    ps->flagRun = 1;
    pthread_create(&ps->th, NULL, &pe_thread, (void*)ps);
    return ps;
}

void pe_exit(PostureStruct **ps)
{
    if(!ps || !(*ps))
        return;
    if ((*ps)->flagRun)
    {
        (*ps)->flagRun = 0;
        pthread_join((*ps)->th, NULL);
        free(*ps);
    }
    (*ps) = NULL;
}

//获取罗盘角度(rad:[-pi, pi])
double pe_dir(void)
{
#if (PE_USE_HMC5883 > 0)
    return (double)hmc5883_get();
#else
    return 0;
#endif
}

