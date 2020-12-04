/*
 *  根据MPU6050数据计算姿态
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

#include "delayus.h"
#include "posture.h"
#include "pseudo3d.h"
#include "pe_math.h"

#include "mpu6050.h"
#include "serialSensor.h"
#include "tcpServer.h"
#include "mma8451.h"
#include "hmc5883.h"

#define POSTURE_PI 3.14159265358979323846

// 传感器5选1
#define SENSOR_MPU6050 //启用mpu6050
// #define SENSOR_SERIALSENSOR //启用serialSensor
// #define SENSOR_TCPSERVER //启用tcpServer
// #define SENSOR_MMA8451 //启用MMA8451
// #define SENSOR_HMC5883 //启用HMC5883罗盘

// 存/读文件(选一个,读文件时不依赖传感器)
// #define PE_SAVE_FILE "./data.txt"
// #define PE_LOAD_FILE "./data.txt"

// 启用四元数解算
#define PE_QUATERNION

// 计算额外受力
#define PE_ACCFORCE

// (unit:N/kg)
#define PE_GRAVITY 9.8
// (unit:kg)
#define PE_MASS 1

// 梯形面积计算方式求积分
#define TRAPEZIOD_SUM(new, old) \
    ((new + old) / 2 * ps->intervalMs / 1000)

void pe_accel(PostureStruct *ps, float *valAcc)
{
    // 三轴向加速度归零差值
    float err = 0.0001, errZ = 0.001;
    float valTmp[3];
    // 用逆矩阵把三轴受力的合向量转为空间坐标系下的向量
    memcpy(valTmp, valAcc, sizeof(float) * 3);
    matrix_zyx(ps->rollXYZ, valTmp);
    // 则该向量在水平方向的分量即为横纵向的g值,空间坐标系三轴向G值(单位:g)
    ps->gX = valTmp[0] + ps->gXErr;
    ps->gY = valTmp[1] + ps->gYErr;
    ps->gZ = valTmp[2] + ps->gZErr;
    // 合受力(单位:g)
    ps->gXYZ = sqrt(ps->gX * ps->gX + ps->gY * ps->gY + ps->gZ * ps->gZ);
#ifdef PE_ACCFORCE
    // 把空间坐标系下的水平分量旋转回物体参考系
    ps->accForce[0] = ps->gX;
    ps->accForce[1] = ps->gY;
    ps->accForce[2] = 0;//ps->gZ;
    matrix_xyz(ps->rollXYZ, ps->accForce);
#endif
    // 得到剔除额外受力(仅受重力)的加速度计数据
    valTmp[0] = valAcc[0] - ps->accForce[0];
    valTmp[1] = valAcc[1] - ps->accForce[1];
    valTmp[2] = valAcc[2] - ps->accForce[2];
    // accel计算姿态
    ps->accRollXYZ[0] = atan2(valTmp[1], valTmp[2]);
    ps->accRollXYZ[1] = -atan2(valTmp[0], sqrt(valTmp[1] * valTmp[1] + valTmp[2] * valTmp[2]));
    ps->accRollXYZ[2] = 0;
    // bakup
    memcpy(ps->accXYZ, valAcc, sizeof(float) * 3);
#if 1
    //test
    if (ps->gX > err)
        ps->gXErr -= err;
    else if (ps->gX < -err)
        ps->gXErr += err;
    if (ps->gY > err)
        ps->gYErr -= err;
    else if (ps->gY < -err)
        ps->gYErr += err;
    if (ps->gZ > errZ)
        ps->gZErr -= errZ;
    else if (ps->gZ < -errZ)
        ps->gZErr += errZ;
#endif
}

void pe_inertial_navigation(PostureStruct *ps)
{
    float speX, speY, speZ, aX, aY, aZ;
    aX = ps->gX * PE_GRAVITY / PE_MASS;
    aY = ps->gY * PE_GRAVITY / PE_MASS;
    aZ = ps->gZ * PE_GRAVITY / PE_MASS;
    //g值积分得到速度
    speX = ps->speX + TRAPEZIOD_SUM(aX, ps->aX);
    speY = ps->speY + TRAPEZIOD_SUM(aY, ps->aY);
    speZ = ps->speZ + TRAPEZIOD_SUM(aZ, ps->aZ);
    //速度积分得到移动距离
    ps->movX += TRAPEZIOD_SUM(speX, ps->speX);
    ps->movY += TRAPEZIOD_SUM(speY, ps->speY);
    ps->movZ += TRAPEZIOD_SUM(speZ, ps->speZ);
    //bakup
    ps->aX = aX;
    ps->aY = aY;
    ps->aZ = aZ;
    ps->speX = speX;
    ps->speY = speY;
    ps->speZ = speZ;
}

//复位(重置计算值)
void pe_reset(PostureStruct *ps)
{
    memset(ps->gyrRollXYZ, 0, sizeof(float) * 3);
    memset(ps->gyrRollXYZ2, 0, sizeof(float) * 3);
    memset(ps->accRollXYZ, 0, sizeof(float) * 3);
    memset(ps->accForce, 0, sizeof(float) * 3);
    ps->rollZErr -= ps->rollXYZ[2];
    ps->aX = ps->aY = ps->aZ = 0;
    ps->speX = ps->speY = ps->speZ = 0;
    ps->movX = ps->movY = ps->movZ = 0;
}

void pe_gyro(PostureStruct *ps, float *valGyr)
{
    // 累加角加速度得到角度值
    ps->gyrRollXYZ2[0] += TRAPEZIOD_SUM(valGyr[0], ps->gyrXYZ[0]);
    ps->gyrRollXYZ2[1] += TRAPEZIOD_SUM(valGyr[1], ps->gyrXYZ[1]);
    ps->gyrRollXYZ2[2] += TRAPEZIOD_SUM(valGyr[2], ps->gyrXYZ[2]);
    // 范围限制
    if (ps->gyrRollXYZ2[0] > 180.00f)
        ps->gyrRollXYZ2[0] -= 360.00f;
    else if (ps->gyrRollXYZ2[0] < -180.00f)
        ps->gyrRollXYZ2[0] += 360.00f;
    // 范围限制
    if (ps->gyrRollXYZ2[1] > 180.00f)
        ps->gyrRollXYZ2[1] -= 360.00f;
    else if (ps->gyrRollXYZ2[1] < -180.00f)
        ps->gyrRollXYZ2[1] += 360.00f;
    // 范围限制
    if (ps->gyrRollXYZ2[2] > 180.00f)
        ps->gyrRollXYZ2[2] -= 360.00f;
    else if (ps->gyrRollXYZ2[2] < -180.00f)
        ps->gyrRollXYZ2[2] += 360.00f;
    // degree to rad
    ps->gyrRollXYZ[0] = ps->gyrRollXYZ2[0] / 180 * POSTURE_PI;
    ps->gyrRollXYZ[1] = ps->gyrRollXYZ2[1] / 180 * POSTURE_PI;
    ps->gyrRollXYZ[2] = ps->gyrRollXYZ2[2] / 180 * POSTURE_PI;
    // copy
    memcpy(ps->gyrXYZ, valGyr, sizeof(float) * 3);
}

void *pe_thread(void *argv)
{
    DELAY_US_INIT;
    int timeCount = 0;
    float valRoll[3] = {0};
    float valGyr[3] = {0};
    float valAcc[3] = {0};
#ifdef PE_QUATERNION
    float valRoll2[3] = {0};
#endif
    PostureStruct *ps = (PostureStruct *)argv;
    //存/读文件数据
#ifdef PE_SAVE_FILE
    FILE *f_fp = fopen(PE_SAVE_FILE, "w");
    if (f_fp)
        fprintf(f_fp, "%dms\r\n", ps->intervalMs);
#endif
#ifdef PE_LOAD_FILE
    int tmpInt;
    FILE *f_fp = fopen(PE_LOAD_FILE, "r");
    if (f_fp)
    {
        if (fscanf(f_fp, "%dms\r\n", &ps->intervalMs) == 1)
            printf("load file interval %dms\r\n", ps->intervalMs);
    }
#endif

    //传感器初始化
#ifdef SENSOR_MPU6050
    if (mpu6050_init(1000 / ps->intervalMs, 0) != 0)
        return NULL;
#endif
#ifdef SENSOR_SERIALSENSOR
    serialSensor_get(valGyr, valAcc);
    DELAY_US(200000);
#endif
#ifdef SENSOR_TCPSERVER
    tcpServer_get(valRoll, valGyr, valAcc);
    DELAY_US(200000);
#endif
#ifdef SENSOR_MMA8451
    //初始化mma8451
    mma8451_init();
#endif
    //周期采样
    while (ps->flagRun)
    {
#ifndef SENSOR_SERIALSENSOR
        DELAY_US(ps->intervalMs * 1000);
#endif

        // 采样
#ifdef SENSOR_MPU6050
        if (mpu6050_angle(valRoll, valGyr, valAcc) == 0)
        {
            ps->rollXYZ[0] = ps->rollXYZ2[0] = valRoll[1];
            ps->rollXYZ[1] = ps->rollXYZ2[1] = valRoll[0];
            ps->rollXYZ[2] = ps->rollXYZ2[2] = valRoll[2] + ps->rollZErr;
        }
#endif
#ifdef SENSOR_SERIALSENSOR
        ps->intervalMs = 10;
        while(serialSensor_get(valGyr, valAcc))
            delayms(2);
#endif
#ifdef SENSOR_TCPSERVER
        if (tcpServer_get(valRoll, valGyr, valAcc) == 0)
        {
            ps->rollXYZ[0] = ps->rollXYZ2[0] = valRoll[1];
            ps->rollXYZ[1] = ps->rollXYZ2[1] = valRoll[0];
            ps->rollXYZ[2] = ps->rollXYZ2[2] = valRoll[2] + ps->rollZErr;
        }
#endif
#ifdef SENSOR_MMA8451
        mma8451_get(valAcc);
#endif

#ifdef PE_LOAD_FILE
        if (f_fp)
        {
            if (fscanf(f_fp, "%f;%f;%f;%f;%f;%f;%f;%f;%f;\r\n",
                       &valAcc[0], &valAcc[1], &valAcc[2],
                       &valGyr[0], &valGyr[1], &valGyr[2],
                       &valRoll[0], &valRoll[1], &valRoll[2]) < 0)
            {
                fseek(f_fp, 0, SEEK_SET);
                fscanf(f_fp, "%dms\r\n", &tmpInt);
            }
            else
            {
                ps->rollXYZ[0] = ps->rollXYZ2[0] = valRoll[1];
                ps->rollXYZ[1] = ps->rollXYZ2[1] = valRoll[0];
                ps->rollXYZ[2] = ps->rollXYZ2[2] = valRoll[2] + ps->rollZErr;
            }
        }
#endif

#ifdef PE_QUATERNION
        quat_pry(valGyr, valAcc, valRoll2, ps->intervalMs);
        // 得到姿态欧拉角,其中绕z轴添加偏差矫正
        ps->rollXYZ[0] = valRoll2[1];
        ps->rollXYZ[1] = valRoll2[0];
        ps->rollXYZ[2] = valRoll2[2] + ps->rollZErr;
#endif

#ifdef PE_SAVE_FILE
        if (f_fp)
        {
            fprintf(f_fp, "%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;\r\n",
                    valAcc[0], valAcc[1], valAcc[2],
                    valGyr[0], valGyr[1], valGyr[2],
                    valRoll[0], valRoll[1], valRoll[2]);
            fflush(f_fp);
        }
#endif

        // gyro:
        pe_gyro(ps, valGyr);
        // accel:
        pe_accel(ps, valAcc);
        // 惯导参数计算
        pe_inertial_navigation(ps);
        //定时获取罗盘数据 & 温度数据
        timeCount += ps->intervalMs;
        if (timeCount >= 200)
        {
            timeCount = 0;
            //罗盘
            if (mpu6050_compass(ps->compassXYZ) == 0)
                ps->dir = atan2(ps->compassXYZ[1], ps->compassXYZ[0]);
            //温度
            mpu6050_temper(&ps->temper);
        }
    }
    return NULL;
}

/*
 *  初始化
 * 
 *  intervalMs: 采样间隔, 越小误差积累越小, 建议值: 2, 5, 10(推荐),20,25,50
 */
PostureStruct *pe_init(int intervalMs)
{
    PostureStruct *ps;
    if (intervalMs < 1)
        return NULL;
    ps = (PostureStruct *)calloc(1, sizeof(PostureStruct));
    ps->intervalMs = intervalMs;
    ps->flagRun = 1;
    pthread_create(&ps->th, NULL, &pe_thread, (void *)ps);
    return ps;
}

void pe_exit(PostureStruct **ps)
{
    if (!ps || !(*ps))
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
float pe_dir(void)
{
#ifdef SENSOR_HMC5883
    return (float)hmc5883_get();
#else
    return 0;
#endif
}
