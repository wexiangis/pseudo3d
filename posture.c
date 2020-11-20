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

// 启用四元数解算
//#define PE_QUATERNION

// 传感器5选1
// #define SENSOR_MPU6050      //启用mpu6050
#define SENSOR_SERIALSENSOR //启用serialSensor
// #define SENSOR_TCPSERVER    //启用tcpServer
// #define SENSOR_MMA8451 //启用MMA8451
// #define SENSOR_HMC5883 //启用HMC5883罗盘

#ifdef SENSOR_SERIALSENSOR
#include "serialSensor.h"
#endif
#ifdef SENSOR_TCPSERVER
#include "tcpServer.h"
#endif
#ifdef SENSOR_MMA8451
#include "mma8451.h"
#endif
#ifdef SENSOR_HMC5883
#include "hmc5883.h"
#endif

// (unit:N/kg)
#define PE_GRAVITY 9.8
// (unit:kg)
#define PE_MASS 1

// 陀螺仪角度积分公式(这里用的梯形面积计算方式)
// 采样间隔 intervalMs 变化时不需再调整参数
#define GYRO_SUM_FUN(new, old) \
    ((double)(new + old) / 2 * ps->intervalMs / 1000)

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

#ifdef PE_QUATERNION
/*
 *  valG: 原始陀螺仪xyz轴输出值
 *  valA: 原始加速度xyz轴输出值
 *  pry: 输出绕xyz轴角度(单位:rad)
 *  intervalMs: 采样间隔(单位:ms)
 */
void quaternion(short *valG, short *valA, double *pry, int intervalMs)
{
    double Kp = 500.0;                                   // 比例增益支配率收敛到加速度计/磁强计
    double Ki = 2.0;                                     // 积分增益支配率的陀螺仪偏见的衔接
    double halfT = (double)intervalMs / 2 / 1000 / 1000; // 采样周期的一半
    static double q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // 四元数的元素，代表估计方向
    static double exInt = 0, eyInt = 0, ezInt = 0;       // 按比例缩小积分误差
    double norm;
    double ax, ay, az;
    double gx, gy, gz;
    double vx, vy, vz;
    double ex, ey, ez;
    // 测量正常化: 向量(ax,ay,az)除以自身模长,即变为单位向量,方向不变
    norm = sqrt(valA[0] * valA[0] + valA[1] * valA[1] + valA[2] * valA[2]);
    ax = (double)valA[0] / norm;
    ay = (double)valA[1] / norm;
    az = (double)valA[2] / norm;
    // 估计方向的重力
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    // 积分误差比例积分增益
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
    // 调整后的陀螺仪测量
    gx = (double)valG[0] + Kp * ex + exInt;
    gy = (double)valG[1] + Kp * ey + eyInt;
    gz = (double)valG[2] + Kp * ez + ezInt;
    // 整合四元数率和正常化
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    // 正常化四元: 向量除以自身模长,即转换为单位向量,方向不变
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    //
    pry[1] = asin(-2 * q1 * q3 + 2 * q0 * q2);                                      // pitch
    pry[0] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1);      // roll
    pry[2] = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3); // yaw
}
#endif

void pe_accel(PostureStruct *ps, short *valA)
{
    double err = 0.0001, errZ = 0.001;
    double gX, gY, gZ;
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
    vXYZ[0] = ps->vAX2;
    vXYZ[1] = ps->vAY2;
    vXYZ[2] = ps->vAZ2;
    rXYZ[0] = ps->rX;
    rXYZ[1] = ps->rY;
    rXYZ[2] = ps->rZ;
    //用逆矩阵把三轴受力的合向量转为空间坐标系下的向量
    p3d_matrix_zyx(rXYZ, vXYZ);
    //则该向量在水平方向的分量即为横纵向的g值
    gX = vXYZ[0] + ps->gXErr;
    gY = vXYZ[1] + ps->gYErr;
    gZ = vXYZ[2] + ps->gZErr;
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
    //bakup
    ps->gX = gX;
    ps->gY = gY;
    ps->gZ = gZ;
    // 合受力(单位:g)
    ps->gXYZ = sqrt(gX * gX + gY * gY + gZ * gZ);
}

void pe_inertial_navigation(PostureStruct *ps)
{
    double speX, speY, speZ, aX, aY, aZ;
    //
    aX = ps->gX * PE_GRAVITY / PE_MASS;
    aY = ps->gY * PE_GRAVITY / PE_MASS;
    aZ = ps->gZ * PE_GRAVITY / PE_MASS;
    //g值积分得到速度
    speX = ps->speX + SPE_SUN_FUN(aX, ps->aX);
    speY = ps->speY + SPE_SUN_FUN(aY, ps->aY);
    speZ = ps->speZ + SPE_SUN_FUN(aZ, ps->aZ);
    //速度积分得到移动距离
    ps->movX += MOV_SUN_FUN(speX, ps->speX);
    ps->movY += MOV_SUN_FUN(speY, ps->speY);
    ps->movZ += MOV_SUN_FUN(speZ, ps->speZ);
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
    ps->rGX = ps->rGY = ps->rGZ = 0;
    ps->rAX = ps->rAY = ps->rAZ = 0;
    ps->rZErr -= ps->rZ;
    ps->aX = ps->aY = ps->aZ = 0;
    ps->speX = ps->speY = ps->speZ = 0;
    ps->movX = ps->movY = ps->movZ = 0;
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
    PostureStruct *ps = (PostureStruct *)argv;
    //传感器初始化
#ifdef SENSOR_MPU6050
    if (mpu6050_init(1000 / ps->intervalMs, 0) != 0)
        return NULL;
#endif
#ifdef SENSOR_SERIALSENSOR
    serialSensor_get(valR, valG, valA);
    DELAY_US(ps->intervalMs * 1000);
#endif
#ifdef SENSOR_TCPSERVER
    tcpServer_get(valR, valG, valA);
    DELAY_US(ps->intervalMs * 1000);
#endif
#ifdef SENSOR_MMA8451
    //初始化mma8451
    mma8451_init();
#endif
    //周期采样
    while (ps->flagRun)
    {
        DELAY_US(ps->intervalMs * 1000);
        // 采样
#ifdef SENSOR_MPU6050
        if (mpu6050_angle(valR, valG, valA) == 0)
        {
            ps->rX = valR[1];
            ps->rY = valR[0];
            ps->rZ = valR[2] + ps->rZErr;
        }
#endif
#ifdef SENSOR_SERIALSENSOR
        if (serialSensor_get(valR, valG, valA) == 0)
        {
            ps->rX = valR[1];
            ps->rY = valR[0];
            ps->rZ = valR[2] + ps->rZErr;
        }
#endif
#ifdef SENSOR_TCPSERVER
        if (tcpServer_get(valR, valG, valA) == 0)
        {
            ps->rX = valR[1];
            ps->rY = valR[0];
            ps->rZ = valR[2] + ps->rZErr;
        }
#endif
#ifdef SENSOR_MMA8451
        mma8451_get(valA);
#endif
#ifdef PE_QUATERNION
        quaternion(valG, valA, valR, ps->intervalMs);
        ps->rX = valR[0];
        ps->rY = valR[1];
        ps->rZ = valR[2] + ps->rZErr;
#endif
        // gyro:
        pe_gyro(ps, valG);
        // accel:
        pe_accel(ps, valA);
        // 惯导参数计算
        pe_inertial_navigation(ps);
        //定时获取罗盘数据 & 温度数据
        timeCount += ps->intervalMs;
        if (timeCount >= 200)
        {
            timeCount = 0;
            //罗盘
            if (mpu6050_compass(valC) == 0)
            {
                ps->vCX = valC[0];
                ps->vCY = valC[1];
                ps->dir = atan2((float)ps->vCY, (float)ps->vCX);
            }
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
PostureStruct *pe_init(unsigned short intervalMs)
{
    PostureStruct *ps = (PostureStruct *)calloc(1, sizeof(PostureStruct));
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
double pe_dir(void)
{
#ifdef SENSOR_HMC5883
    return (double)hmc5883_get();
#else
    return 0;
#endif
}
