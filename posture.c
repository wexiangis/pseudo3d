/*
 *  根据MPU6050数据计算姿态
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "posture.h"
#include "delayus.h"
#include "inv_mpu.h"
#include "pseudo3d.h"

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

// 陀螺仪积分矫正倍数
#define GYRO_REDUCE_POW 1000

// 陀螺仪角度积分公式(这里用的梯形面积计算方式)
// 采样间隔 intervalMs 变化时不需再调整参数
#define GYRO_SUM_FUN(new, old) \
(((double)new - (double)(new - old) / 2) * ps->intervalMs / 1000 / GYRO_REDUCE_POW)

// 速度积分矫正倍数
#define SPE_REDUCE_POW 100

// 速度积分得到移动距离(同上面 GYRO_SUM_FUN() )
#define SPE_SUN_FUN(new, old) \
((new - (new - old) / 2) * ps->intervalMs / 1000 / SPE_REDUCE_POW)

void pe_inertial_navigation(PostureStruct *ps)
{
    double xSpe, ySpe, xG, yG;
    double vXYZ[3], rXYZ[3];
    //
    vXYZ[0] = ps->aXG; vXYZ[1] = ps->aYG; vXYZ[2] = ps->aZG;
    rXYZ[0] = ps->rX; rXYZ[1] = ps->rY; rXYZ[2] = ps->rZ;
    //用逆矩阵把三轴受力的合向量转为空间坐标系下的向量
    p3d_matrix_zyx(rXYZ, vXYZ);
    //则该向量在水平方向的分量即为横纵向的g值
    xG = vXYZ[0];
    yG = vXYZ[1];
    //g值积分得到速度
    xSpe = ps->xSpe;
    ySpe = ps->ySpe;
    xSpe += xG / 100;//SPE_SUN_FUN(xG, ps->xG) * 9.8;
    ySpe += yG / 100;//SPE_SUN_FUN(yG, ps->yG) * 9.8;
    ps->xG = xG;
    ps->yG = yG;
    //速度积分得到移动距离
    ps->xMov += SPE_SUN_FUN(xSpe, ps->xSpe);
    ps->yMov += SPE_SUN_FUN(ySpe, ps->ySpe);
    ps->xSpe = xSpe;
    ps->ySpe = ySpe;
}

void *pe_thread(void *argv)
{
    DELAY_US_INIT;
    double dVal[3];
    short agVal[3], acVal[3];
    PostureStruct *ps = (PostureStruct*)argv;
    //初始化mpu6050
    if (mpu_dmp_init(1000 / ps->intervalMs, 0) != 0)
        return NULL;
    //周期采样
    while (ps->flagRun)
    {
        DELAY_US(ps->intervalMs * 1000);
        // 采样
        if (mpu_dmp_get_data(dVal, agVal, acVal) == 0)
        {
            // dmp库用4元素法得到的欧拉角(单位:rad)
            ps->rX = dVal[1];
            ps->rY = dVal[0];
            ps->rZ = dVal[2] + ps->zErr;
        }
        // 累加角加速度得到角度值
        ps->gX += GYRO_SUM_FUN(agVal[0], ps->gXVal);
        ps->gY += GYRO_SUM_FUN(agVal[1], ps->gYVal);
        ps->gZ += GYRO_SUM_FUN(agVal[2], ps->gZVal);
        // 范围限制
        if (ps->gX > PE_PI)
            ps->gX -= PE_2PI;
        else if (ps->gX < -PE_PI)
            ps->gX += PE_2PI;
        if (ps->gY > PE_PI)
            ps->gY -= PE_2PI;
        else if (ps->gY < -PE_PI)
            ps->gY += PE_2PI;
        if (ps->gZ > PE_PI)
            ps->gZ -= PE_2PI;
        else if (ps->gZ < -PE_PI)
            ps->gZ += PE_2PI;
        // copy
        ps->gXVal = agVal[0];
        ps->gYVal = agVal[1];
        ps->gZVal = agVal[2];
        ps->aXVal = acVal[0];
        ps->aYVal = acVal[1];
        ps->aZVal = acVal[2];
        // accel计算姿态
        ps->aX = atan2((double)ps->aYVal, (double)ps->aZVal);
        ps->aY = -atan2((double)ps->aXVal,
            (double)sqrt((double)ps->aYVal * ps->aYVal + (double)ps->aZVal * ps->aZVal));
        ps->aZ = 0;
        // 角速度转换(单位:rad/s)
        ps->gXR = (double)ps->gXVal / GYRO_VAL_P_RED;
        ps->gYR = (double)ps->gYVal / GYRO_VAL_P_RED;
        ps->gZR = (double)ps->gZVal / GYRO_VAL_P_RED;
        // 各轴向受力转换(单位:g)
        ps->aXG = (double)ps->aXVal / ACCEL_VAL_P_G;
        ps->aYG = (double)ps->aYVal / ACCEL_VAL_P_G;
        ps->aZG = (double)ps->aZVal / ACCEL_VAL_P_G;
        // 合受力(单位:g)
        ps->aG = sqrt(ps->aXG * ps->aXG + ps->aYG * ps->aYG + ps->aZG * ps->aZG);
        // 惯导参数计算
        pe_inertial_navigation(ps);
    }

    return NULL;
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

//复位(重置计算值)
void pe_reset(PostureStruct *ps)
{
    ps->gX = ps->gY = ps->gZ = 0;
    ps->aX = ps->aY = ps->aZ = 0;
    ps->zErr -= ps->rZ;
    ps->xSpe = ps->ySpe = ps->xMov = ps->yMov = 0;
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

