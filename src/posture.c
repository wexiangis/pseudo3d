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

#define POSTURE_PI 3.14159265358979323846

// 传感器5选1
#define SENSOR_MPU6050 //启用mpu6050
// #define SENSOR_SERIALSENSOR //启用serialSensor
// #define SENSOR_TCPSERVER //启用tcpServer
// #define SENSOR_MMA8451 //启用MMA8451

// 存/读文件(选一个,读文件时不依赖传感器)
// #define PE_SAVE_FILE "./data.txt"
// #define PE_LOAD_FILE "./data.txt"

// 启用四元数解算
#define PE_QUATERNION

//使能双子样处理(姿态更新间隔将由10ms变为20ms)
// #define TWO_SAMPLE

// (unit:N/kg)
#define PE_GRAVITY 9.8
// (unit:kg)
#define PE_MASS 1

// 梯形面积计算方式求积分
#define TRAPEZIOD_SUM(new, old) \
    ((new + old) / 2 * ps->intervalMs / 1000)

void pe_accel(PostureStruct *ps, float valAcc[3])
{
    // 三轴向加速度归零差值
    float err = 0.0001;
    //合受力
    ps->accXYZ2 = sqrt(valAcc[0] * valAcc[0] + valAcc[1] * valAcc[1] + valAcc[2] * valAcc[2]);
    // 用逆矩阵把三轴受力的合向量转为空间坐标系下的向量
    matrix_zyx(ps->rollXYZ, valAcc, ps->gForce);
    // 则该向量在水平方向的分量即为横纵向的g值,空间坐标系三轴向G值(单位:g)
    ps->gXYZ[0] = ps->gForce[0] + ps->gXYZErr[0];
    ps->gXYZ[1] = ps->gForce[1] + ps->gXYZErr[1];
    ps->gXYZ[2] = ps->gForce[2] + ps->gXYZErr[2] - 1.0;
    // accel计算姿态
    // ps->accRollXYZ[0] = atan2(ps->gravity[1], ps->gravity[2]);
    // ps->accRollXYZ[1] = -atan2(ps->gravity[0],
    //     sqrt(ps->gravity[1] * ps->gravity[1] + ps->gravity[2] * ps->gravity[2]));
    ps->accRollXYZ[0] = atan2(valAcc[1], valAcc[2]);
    ps->accRollXYZ[1] = -atan2(valAcc[0], sqrt(valAcc[1] * valAcc[1] + valAcc[2] * valAcc[2]));
    ps->accRollXYZ[2] = 0;
#if 0
    //test
    if (ps->gXYZ[0] > err)
        ps->gXYZErr[0] -= err;
    else if (ps->gXYZ[0] < -err)
        ps->gXYZErr[0] += err;
    if (ps->gXYZ[1] > err)
        ps->gXYZErr[1] -= err;
    else if (ps->gXYZ[1] < -err)
        ps->gXYZErr[1] += err;
    if (ps->gXYZ[2] > err)
        ps->gXYZErr[2] -= err;
    else if (ps->gXYZ[2] < -err)
        ps->gXYZErr[2] += err;
#endif
    // bakup
    memcpy(ps->accXYZ, valAcc, sizeof(float) * 3);
}

void pe_gyro(PostureStruct *ps, float *valGyr)
{
    // copy
    memcpy(ps->gyrXYZ, valGyr, sizeof(float) * 3);
    //使用四元数微分方程来累积陀螺仪角度
    quat_pry(ps->quat_err2, valGyr, NULL, ps->gyrRollXYZ, ps->intervalMs, ps->miscRate);
}

void pe_navigation(PostureStruct *ps)
{
    float aXYZ[3], speXYZ[3];
    aXYZ[0] = ps->gXYZ[0] * PE_GRAVITY / PE_MASS;
    aXYZ[1] = ps->gXYZ[1] * PE_GRAVITY / PE_MASS;
    aXYZ[2] = ps->gXYZ[2] * PE_GRAVITY / PE_MASS;
    //g值积分得到速度
    speXYZ[0] = ps->speXYZ[0] + TRAPEZIOD_SUM(aXYZ[0], ps->aXYZ[0]);
    speXYZ[1] = ps->speXYZ[1] + TRAPEZIOD_SUM(aXYZ[1], ps->aXYZ[1]);
    speXYZ[2] = ps->speXYZ[2] + TRAPEZIOD_SUM(aXYZ[2], ps->aXYZ[2]);
    //速度积分得到移动距离
    ps->movXYZ[0] += TRAPEZIOD_SUM(speXYZ[0], ps->speXYZ[0]);
    ps->movXYZ[1] += TRAPEZIOD_SUM(speXYZ[1], ps->speXYZ[1]);
    ps->movXYZ[2] += TRAPEZIOD_SUM(speXYZ[2], ps->speXYZ[2]);
    //bakup
    memcpy(ps->aXYZ, aXYZ, sizeof(float) * 3);
    memcpy(ps->speXYZ, speXYZ, sizeof(float) * 3);
}

void _pe_stack(float *bak, int len, float new, float *aver)
{
    float sum;
    int i;
    for (i = sum = 0; i < len - 1; i++)
    {
        bak[i] = bak[i + 1];
        sum += bak[i];
    }
    bak[i] = new;
    sum += new;
    if (aver)
        *aver = sum / len;
}

//变化率计算
void pe_changeRate(PostureStruct *ps, float valGyr[3], float valAcc[3])
{
    static float miscRate[10] = {0};
    float g1[3], g2[3], ret[3];
    float aver;
    //角速度变化率(角速度向量的模)
    ps->gyrRate = vector_norm(valGyr);
    //加速度变化率(前后加速度向量的叉乘,再取模)
    vector_to_unit(ps->accXYZ, g1);
    vector_to_unit(valAcc, g2);
    vector_cross_product(g1, g2, ret);
    ps->accRate = vector_norm(ret);
    //综合变化率
    ps->miscRate = ps->gyrRate * ps->accRate;
    //静止标志
    _pe_stack(miscRate, 10, ps->miscRate, &aver);
    if (aver < 0.05)
        ps->quiet = true;
    else
        ps->quiet = false;
}

//角速度误差矫正
void pe_gyrCorrect(PostureStruct *ps, float valGyr[3])
{
    static float valGyrBak[3][10] = {0};
    float aver[3];
    //
    _pe_stack(valGyrBak[0], 10, valGyr[0], &aver[0]);
    _pe_stack(valGyrBak[1], 10, valGyr[1], &aver[1]);
    _pe_stack(valGyrBak[2], 10, valGyr[2], &aver[2]);
    //
    // if (ps->quiet)
    // {
    //     ps->gyrXYZErr[0] = -aver[0];
    //     ps->gyrXYZErr[1] = -aver[1];
    //     ps->gyrXYZErr[2] = -aver[2];
    // }
    //
    valGyr[0] += ps->gyrXYZErr[0];
    valGyr[1] += ps->gyrXYZErr[1];
    valGyr[2] += ps->gyrXYZErr[2];
}

/*等效旋转矢量转换为四元数 */
void rv2q(float rv[3], float quat[4])
{
#define F1 (2 * 1) // define: Fk=2^k*k!
#define F2 (F1 * 2 * 2)
#define F3 (F2 * 2 * 3)
#define F4 (F3 * 2 * 4)
#define F5 (F4 * 2 * 5)
    float n2 = rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2];
    float c, f;
    float n4, n_2;

    if (n2 < (POSTURE_PI / 180.0 * POSTURE_PI / 180.0)) // 0.017^2
    {
        n4 = n2 * n2;
        c = 1.0 - n2 * (1.0 / F2) + n4 * (1.0 / F4);
        f = 0.5 - n2 * (1.0 / F3) + n4 * (1.0 / F5);
    }
    else
    {
        n_2 = sqrt(n2) / 2.0;
        c = cos(n_2);
        f = sin(n_2) / n_2 * 0.5;
    }
    quat[0] = c;
    quat[1] = rv[0] * f;
    quat[2] = rv[1] * f;
    quat[3] = rv[2] * f;
}

void q2att(float qnb[4], float pry[3])
{
	float	q11 = qnb[0]*qnb[0], q12 = qnb[0]*qnb[1], q13 = qnb[0]*qnb[2], q14 = qnb[0]*qnb[3], 
			q22 = qnb[1]*qnb[1], q23 = qnb[1]*qnb[2], q24 = qnb[1]*qnb[3],     
			q33 = qnb[2]*qnb[2], q34 = qnb[2]*qnb[3],  
			q44 = qnb[3]*qnb[3];

	pry[0] = asin(2*(q34+q12));
	pry[1] = atan2(-2*(q24-q13), q11-q22-q33+q44);
	pry[2] = atan2(-2*(q23-q14), q11-q22+q33-q44);
}

//复位(重置计算值)
void pe_reset(PostureStruct *ps)
{
    memset(ps->gyrRollXYZ, 0, sizeof(float) * 3);
    memset(ps->accRollXYZ, 0, sizeof(float) * 3);
    memset(ps->gForce, 0, sizeof(float) * 3);
    memset(ps->gXYZ, 0, sizeof(float) * 3);
    memset(ps->aXYZ, 0, sizeof(float) * 3);
    memset(ps->speXYZ, 0, sizeof(float) * 3);
    memset(ps->movXYZ, 0, sizeof(float) * 3);
#if 1
    ps->rollZErr -= ps->rollXYZ[2];
#else
    memset(ps->quat_err, 0, sizeof(float) * 7);
    ps->quat_err[0] = 1.0f;
#endif
    memset(ps->quat_err2, 0, sizeof(float) * 7);
    ps->quat_err2[0] = 1.0f;
}

void *pe_thread(void *argv)
{
    DELAY_US_INIT;
    float valRoll[3] = {0};
    float valGyr[6] = {0};
    float valAA[2] = {0};
    float valAcc[3] = {0};
#ifdef PE_QUATERNION
    float valRoll2[3] = {0};
#endif
    PostureStruct *ps = (PostureStruct *)argv;
#ifdef TWO_SAMPLE
    int ts_count = 0;
    float aa[3][3] = {0};
    float qh[4] = {0};
    float rv[3] = {0};
    float *quat;
#endif

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
    serialSensor_get(valGyr, valAcc, NULL, valAA, NULL, NULL, NULL);
#endif
#ifdef SENSOR_TCPSERVER
    tcpServer_get(valRoll, valGyr, valAcc);
    delayms(200);
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
#else
        valGyr[3] = valGyr[0];
        valGyr[4] = valGyr[1];
        valGyr[5] = valGyr[2];

        ps->intervalMs = 10;
        while (serialSensor_get(valGyr, valAcc, NULL, valAA, NULL, NULL, NULL))
            delayus(50);
        valAcc[0] /= -PE_GRAVITY;
        valAcc[1] /= -PE_GRAVITY;
        valAcc[2] /= PE_GRAVITY;
#endif
        // 采样
#ifdef SENSOR_MPU6050
        if (mpu6050_angle(valRoll, valGyr, valAcc) == 0)
        {
            ps->rollXYZ[0] = valRoll[1];
            ps->rollXYZ[1] = valRoll[0];
            ps->rollXYZ[2] = valRoll[2] + ps->rollZErr;
        }
#endif
#ifdef SENSOR_TCPSERVER
        if (tcpServer_get(valRoll, valGyr, valAcc) == 0)
        {
            ps->rollXYZ[0] = valRoll[1];
            ps->rollXYZ[1] = valRoll[0];
            ps->rollXYZ[2] = valRoll[2] + ps->rollZErr;
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
                ps->rollXYZ[0] = valRoll[1];
                ps->rollXYZ[1] = valRoll[0];
                ps->rollXYZ[2] = valRoll[2] + ps->rollZErr;
            }
        }
#endif
        //变化率
        pe_changeRate(ps, valGyr, valAcc);
        //角速度矫正
        // pe_gyrCorrect(ps, valGyr);

#ifdef TWO_SAMPLE

        pe_accel(ps, valAcc);

        aa[ts_count][0] = valAA[1];// * 0.01;
        aa[ts_count][1] = valAA[0];// * 0.01;
        aa[ts_count][2] = valAA[2];// * 0.01;

        if (++ts_count < 1)
            continue;

        if (ts_count == 1)
        {
            rv[0] = (aa[0][0] + aa[0][0] * aa[1][0] * 1 / 12) * 1.8;
            rv[1] = (aa[0][1] + aa[0][1] * aa[1][1] * 1 / 12) * 1.8;
            rv[2] = (aa[0][2] + aa[0][2] * aa[1][2] * 1 / 12) * 1.8;

            // rv[0] = (aa[0][0] + 0) * 2;
            // rv[1] = (aa[0][1] + 0) * 2;
            // rv[2] = (aa[0][2] + 0) * 2;

            aa[1][0] = aa[0][0];
            aa[1][1] = aa[0][1];
            aa[1][2] = aa[0][2];
        }
        else
        {
            rv[0] = (aa[0][0] + aa[1][0] + aa[0][0] * aa[1][0] * 2 / 3) * 1.8;
            rv[1] = (aa[0][1] + aa[1][1] + aa[0][1] * aa[1][1] * 2 / 3) * 1.8;
            rv[2] = (aa[0][2] + aa[1][2] + aa[0][2] * aa[1][2] * 2 / 3) * 1.8;
        }
        ts_count = 0;

        rv2q(rv, qh);

        quat = ps->quat_err2;
        // quat = qh;

        // quat_multiply(quat, qh, quat);
        // memcpy(quat, qh, sizeof(float) * 4);

        // vector_to_unit2(quat, quat);

        // rv[0] = aa[0][0] / 0.01;
        // rv[1] = aa[0][1] / 0.01;
        // rv[2] = aa[0][2] / 0.01;
        // quat_pry(quat, rv, NULL, valRoll2, ps->intervalMs, ps->miscRate);

        // quat_to_pry(quat, valRoll2);
        q2att(quat, valRoll2);
        ps->rollXYZ[0] = valRoll2[0];
        ps->rollXYZ[1] = valRoll2[1];
        ps->rollXYZ[2] = valRoll2[2] + ps->rollZErr;
#elif 0

        aa[ts_count][0] = (valAA[1] + 0.001);
        aa[ts_count][1] = (valAA[0] + 0.001);
        aa[ts_count][2] = (valAA[2] + 0.001);

        if (++ts_count < 2)
            continue;
        ts_count = 0;

        valAA[0] = aa[0][0] + aa[1][0]; // + (aa[0][0] / 140 - aa[1][0] * 13 / 210 + aa[0][0] * 323 / 420) * aa[1][0];
        valAA[1] = aa[0][1] + aa[1][1]; // + (aa[0][1] / 140 - aa[1][1] * 13 / 210 + aa[0][1] * 323 / 420) * aa[1][1];
        valAA[2] = aa[0][2] + aa[1][2]; // + (aa[0][2] / 140 - aa[1][2] * 13 / 210 + aa[0][2] * 323 / 420) * aa[1][2];

        valGyr[0] = valAA[0] * 1000 / ps->intervalMs / 2;
        valGyr[1] = valAA[1] * 1000 / ps->intervalMs / 2;
        valGyr[2] = valAA[2] * 1000 / ps->intervalMs / 2;

        quat_pry(ps->quat_err, valGyr, NULL, valRoll2, ps->intervalMs * 2, ps->miscRate);
        // 得到姿态欧拉角,其中绕z轴添加偏差矫正
        ps->rollXYZ[0] = valRoll2[0];
        ps->rollXYZ[1] = valRoll2[1];
        ps->rollXYZ[2] = valRoll2[2] + ps->rollZErr;
#else
#ifdef PE_QUATERNION
        quat_pry(ps->quat_err, valGyr, valAcc, valRoll2, ps->intervalMs, ps->miscRate);
        // 得到姿态欧拉角,其中绕z轴添加偏差矫正
        ps->rollXYZ[0] = valRoll2[0];
        ps->rollXYZ[1] = valRoll2[1];
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
        pe_navigation(ps);
#endif

        //
        if (ps->callback)
            ps->callback(ps->obj);
    }
    return NULL;
}

/*
 *  初始化
 * 
 *  intervalMs: 采样间隔, 越小误差积累越小, 建议值: 2, 5, 10(推荐),20,25,50
 */
PostureStruct *pe_init(int intervalMs, void *obj, void (*callback)(void *))
{
    PostureStruct *ps;
    if (intervalMs < 1)
        return NULL;
    ps = (PostureStruct *)calloc(1, sizeof(PostureStruct));
    ps->obj = obj;
    ps->callback = callback;
    ps->intervalMs = intervalMs;
    ps->flagRun = 1;
    ps->quat_err[0] = 1.0f;
    ps->quat_err2[0] = 1.0f;
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
    return 0;
}
