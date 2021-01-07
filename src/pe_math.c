#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MATH_PI 3.14159265358979323846

/*
 *  quaternion解算
 *  参数:
 *      quat_err[10]: 四元数和误差积累数组,初始值用 {1,0,0,0,0,0,0,0,0,0} (必要参数)
 *      valG[3]: 陀螺仪xyz轴输出,单位:deg/s (必要参数)
 *      valA[3]: 加速度xyz轴输出,单位:g  (可以置NULL,等于纯陀螺仪计算姿态)
 *      pry[3]: 输出绕xyz轴角度,单位:rad (可以置NULL)
 *      intervalMs: 采样间隔,单位:ms (必要参数)
 *      miscRate: xxx
 */
void quat_pry(float quat_err[10], float valG[3], float valA[3], float pry[3], int intervalMs, float miscRate)
{
    float Kp = 2.0f;
    float Ki = 0.001f;
    // 时间间隔一半, 后面 pi/180 用于 deg/s 转 rad/s
    float halfT = (float)intervalMs / 2 / 1000;
    float q[4];
    float eInt[3];
    float *errTotal;
    float norm;
    float ax, ay, az;
    float gx, gy, gz;
    float vx, vy, vz;
    float ex, ey, ez;
    if (!valG)
        return;
    // stack out
    memcpy(q, &quat_err[0], sizeof(float) * 4);
    memcpy(eInt, &quat_err[4], sizeof(float) * 3);
    errTotal = &quat_err[7];
    // 估计重力的方向,即(0,0,1)向量经过四元数的"逆旋转",即Crb
    vx = ax = 2 * (q[1] * q[3] - q[0] * q[2]);
    vy = ay = 2 * (q[0] * q[1] + q[2] * q[3]);
    vz = az = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    // 加速度向量转为单位向量
    if (valA)
    {
        norm = sqrt(valA[0] * valA[0] + valA[1] * valA[1] + valA[2] * valA[2]);
        if (!isnan(norm))
        {
            ax = valA[0] / norm;
            ay = valA[1] / norm;
            az = valA[2] / norm;
        }
        // 动态参数
        if (norm < 0.8 || norm > 1.2)
            Kp = Ki = 0;
        else if (norm < 1.0)
        {
            Kp = (norm - 0.8) / 0.2 * Kp;
            Ki = (norm - 0.8) / 0.2 * Ki;
        }
        else if (norm > 1.0)
        {
            Kp = (1.2 - norm) / 0.2 * Kp;
            Ki = (1.2 - norm) / 0.2 * Ki;
        }
        // if (miscRate >= 110)
        // {
        //     Kp += Kp * 100;
        //     Ki += Ki * 100;
        // }
        // else if (miscRate > 10)
        // {
        //     Kp += (miscRate - 10) * Kp;
        //     Ki += (miscRate - 10) * Ki;
        // }
        // else if (miscRate < 0.01)
        // {
        //     Kp += (0.01 - miscRate) * 10000 * Kp;
        //     Ki += (0.01 - miscRate) * 10000 * Ki;
        // }
    }
    // 叉积补偿滤波(互补滤波) https://blog.csdn.net/weixin_40378598/article/details/108133032
    // 加速度向量(ax, ay, az)和向量(vx, vz, vy)的叉乘, gxyz 和 vxyz 夹角越大(90度时最大)则 exyz 值越大
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    // 积分误差比例积分增益
    eInt[0] += ex * Ki;
    eInt[1] += ey * Ki;
    eInt[2] += ez * Ki;
    //
    errTotal[0] = Kp * ex + eInt[0];
    errTotal[1] = Kp * ey + eInt[1];
    errTotal[2] = Kp * ez + eInt[2];
    // 调整后的陀螺仪测量
#if 1
    gx = valG[0] * MATH_PI / 180 + errTotal[0];
    gy = valG[1] * MATH_PI / 180 + errTotal[1];
    gz = valG[2] * MATH_PI / 180 + errTotal[2];
#else
    gx = valG[0] * MATH_PI / 180;
    gy = valG[1] * MATH_PI / 180;
    gz = valG[2] * MATH_PI / 180;
#endif
    // 四元数微分方程
    q[0] += (-q[1] * gx - q[2] * gy - q[3] * gz) * halfT;
    q[1] += (q[0] * gx + q[2] * gz - q[3] * gy) * halfT;
    q[2] += (q[0] * gy - q[1] * gz + q[3] * gx) * halfT;
    q[3] += (q[0] * gz + q[1] * gy - q[2] * gx) * halfT;
    // 单位化
    norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (isnan(norm))
        return;
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
    // pry
    if (pry)
    {
        pry[1] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]);
        pry[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1);
        pry[2] = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    }
    // stack in
    memcpy(&quat_err[0], q, sizeof(float) * 4);
    memcpy(&quat_err[4], eInt, sizeof(float) * 3);
}

// 四元数角增量(龙格塔微分方程)
void quat_diff(float q[4], float roll_xyz[3])
{
    float norm;
    q[0] += (-q[1] * roll_xyz[0] - q[2] * roll_xyz[1] - q[3] * roll_xyz[2]) / 2;
    q[1] += (q[0] * roll_xyz[0] + q[2] * roll_xyz[2] - q[3] * roll_xyz[1]) / 2;
    q[2] += (q[0] * roll_xyz[1] - q[1] * roll_xyz[2] + q[3] * roll_xyz[0]) / 2;
    q[3] += (q[0] * roll_xyz[2] + q[1] * roll_xyz[1] - q[2] * roll_xyz[0]) / 2;
    // 单位化
    norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!isnan(norm))
    {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}
// roll_xyz使用单位: 度
void quat_diff2(float q[4], float roll_xyz[3])
{
    float _roll_xyz[3];
    _roll_xyz[0] = roll_xyz[0] * MATH_PI / 180;
    _roll_xyz[1] = roll_xyz[1] * MATH_PI / 180;
    _roll_xyz[2] = roll_xyz[2] * MATH_PI / 180;
    quat_diff(q, _roll_xyz);
}

// 四元数乘法
void quat_multiply(float q1[4], float q2[4], float ret[4])
{
    float _ret[4];
    _ret[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    _ret[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    _ret[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    _ret[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    ret[0] = _ret[0];
    ret[1] = _ret[1];
    ret[2] = _ret[2];
    ret[3] = _ret[3];
}

// 欧拉角转四元数(zyx顺序)
void pry_to_quat(float pry[3], float q[4])
{
    float Qp[4] = {0};
    float Qr[4] = {0};
    float Qy[4] = {0};
    float norm;

    Qp[0] = cos(pry[0] / 2);
    Qp[1] = sin(pry[0] / 2);

    Qr[0] = cos(pry[1] / 2);
    Qr[2] = sin(pry[1] / 2);

    Qy[0] = cos(pry[2] / 2);
    Qy[3] = sin(pry[2] / 2);

    quat_multiply(Qy, Qr, q);
    quat_multiply(q, Qp, q);
    // 单位化
    norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!isnan(norm))
    {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}
// pry使用单位: 度
void pry_to_quat2(float pry[3], float q[4])
{
    float _pry[3];
    _pry[0] = pry[0] * MATH_PI / 180;
    _pry[1] = pry[1] * MATH_PI / 180;
    _pry[2] = pry[2] * MATH_PI / 180;
    pry_to_quat(_pry, q);
}

// 四元数转欧拉角
void quat_to_pry(float q[4], float pry[3])
{
    pry[1] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]);
    pry[0] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1);
    pry[2] = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
}
// pry使用单位: 度
void quat_to_pry2(float q[4], float pry[3])
{
    quat_to_pry(q, pry);
    pry[0] *= 180 / MATH_PI;
    pry[1] *= 180 / MATH_PI;
    pry[2] *= 180 / MATH_PI;
}

/*
 *  四元数方式旋转和逆旋转
 *  参数:
 *      quat[4]: 使用已有的四元数(可置NULL), 将不使用 roll_vector 和 roll_rad
 *      roll_vector[3]: 要绕转的空间向量,右手旋转,大拇指向量方向
 *      roll_rad: 旋转角度,单位:rad
 *      vector[3]: 被旋转的向量,输出结果覆写到此
 *      T: 转置
 */
void quat_roll(float quat[4], float roll_vector[3], float roll_rad, float vector[3], bool T)
{
    float q[4], qT[4];
    float rv[3];
    float v[4], ret[4];
    float norm;

    if (quat)
    {
        memcpy(q, quat, sizeof(float) * 4);
        // 单位化
        norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (!isnan(norm))
        {
            q[0] /= norm;
            q[1] /= norm;
            q[2] /= norm;
            q[3] /= norm;
        }
    }
    else
    {
        //对旋转轴进行单位向量处理(否则旋转后会附带缩放效果)
        memcpy(rv, roll_vector, sizeof(float) * 3);
        //单位化
        norm = sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
        if (!isnan(norm))
        {
            rv[0] /= norm;
            rv[1] /= norm;
            rv[2] /= norm;
        }
        //
        q[0] = cos(roll_rad / 2);
        q[1] = sin(roll_rad / 2) * rv[0];
        q[2] = sin(roll_rad / 2) * rv[1];
        q[3] = sin(roll_rad / 2) * rv[2];
    }

    qT[0] = q[0];
    qT[1] = -q[1];
    qT[2] = -q[2];
    qT[3] = -q[3];

    v[0] = 0;
    v[1] = vector[0];
    v[2] = vector[1];
    v[3] = vector[2];

    if (T)
    {
        quat_multiply(qT, v, ret);
        quat_multiply(ret, q, ret);
    }
    else
    {
        quat_multiply(q, v, ret);
        quat_multiply(ret, qT, ret);
    }

    memcpy(vector, &ret[1], sizeof(float) * 3);
}

static void _quat_roll_xyz(float roll_xyz[3], float xyz[3], float retXyz[3], bool zyx)
{
    float qx[4] = {0}, qy[4] = {0}, qz[4] = {0};
    float qxT[4] = {0}, qyT[4] = {0}, qzT[4] = {0};
    float v[4], ret[4];

    qx[0] = qxT[0] = cos(roll_xyz[0] / 2);
    qx[1] = sin(roll_xyz[0] / 2);
    qxT[1] = -qx[1];

    qy[0] = qyT[0] = cos(roll_xyz[1] / 2);
    qy[2] = sin(roll_xyz[1] / 2);
    qyT[2] = -qy[2];

    qz[0] = qzT[0] = cos(roll_xyz[2] / 2);
    qz[3] = sin(roll_xyz[2] / 2);
    qzT[3] = -qz[3];

    v[0] = 0;
    v[1] = xyz[0];
    v[2] = xyz[1];
    v[3] = xyz[2];

    if (zyx)
    {
        quat_multiply(qz, qy, ret);
        quat_multiply(ret, qx, ret);
        quat_multiply(ret, v, ret);
        quat_multiply(ret, qxT, ret);
        quat_multiply(ret, qyT, ret);
        quat_multiply(ret, qzT, ret);

        // quat_multiply(qxT, qyT, ret);
        // quat_multiply(ret, qzT, ret);
        // quat_multiply(ret, v, ret);
        // quat_multiply(ret, qz, ret);
        // quat_multiply(ret, qy, ret);
        // quat_multiply(ret, qx, ret);
    }
    else
    {
        quat_multiply(qx, qy, ret);
        quat_multiply(ret, qz, ret);
        quat_multiply(ret, v, ret);
        quat_multiply(ret, qzT, ret);
        quat_multiply(ret, qyT, ret);
        quat_multiply(ret, qxT, ret);
    }

    memcpy(retXyz, &ret[1], sizeof(float) * 3);
}

/*
 *  四元数依次三轴旋转
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:rad
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void quat_xyz(float roll_xyz[3], float xyz[3], float retXyz[3])
{
    _quat_roll_xyz(roll_xyz, xyz, retXyz, false);
}
void quat_zyx(float roll_xyz[3], float xyz[3], float retXyz[3])
{
    _quat_roll_xyz(roll_xyz, xyz, retXyz, true);
}

/*
 *  使用现有四元数进行旋转矩阵运算
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:rad
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void quat_matrix_xyz(float quat[4], float xyz[3], float retXyz[3])
{
    float q0 = quat[0];
    float q1 = quat[1];
    float q2 = quat[2];
    float q3 = quat[3];
    float norm;

    // 单位化
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (!isnan(norm))
    {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }

    retXyz[0] =
        xyz[0] * (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) +
        xyz[1] * 2 * (q1 * q2 - q0 * q3) +
        xyz[2] * 2 * (q1 * q3 + q0 * q2);
    retXyz[1] =
        xyz[0] * 2 * (q1 * q2 + q0 * q3) +
        xyz[1] * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) +
        xyz[2] * 2 * (q2 * q3 + q0 * q1);
    retXyz[2] =
        xyz[0] * 2 * (q1 * q3 - q0 * q2) +
        xyz[1] * 2 * (q1 * q2 + q0 * q3) +
        xyz[2] * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}
void quat_matrix_zyx(float quat[4], float xyz[3], float retXyz[3])
{
    float q0 = quat[0];
    float q1 = quat[1];
    float q2 = quat[2];
    float q3 = quat[3];
    float norm;
    
    // 单位化
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (!isnan(norm))
    {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }

    retXyz[0] =
        xyz[0] * (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) +
        xyz[1] * 2 * (q1 * q2 + q0 * q3) +
        xyz[2] * 2 * (q1 * q3 - q0 * q2);
    retXyz[1] =
        xyz[0] * 2 * (q1 * q2 - q0 * q3) +
        xyz[1] * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) +
        xyz[2] * 2 * (q2 * q3 + q0 * q1);
    retXyz[2] =
        xyz[0] * 2 * (q1 * q3 + q0 * q2) +
        xyz[1] * 2 * (q2 * q3 + q0 * q1) +
        xyz[2] * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}

/*
 *  向量叉乘, v1 x v2 = ret
 */
void vector_cross_product(float v1[3], float v2[3], float ret[3])
{
    float _ret[3];
    _ret[0] = v1[1] * v2[2] - v1[2] * v2[1];
    _ret[1] = v1[2] * v2[0] - v1[0] * v2[2];
    _ret[2] = v1[0] * v2[1] - v1[1] * v2[0];
    memcpy(ret, _ret, sizeof(float) * 3);
}

/*
 *  向量取模
 */
float vector_norm(float v[3])
{
    float ret = 0;
    ret = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (isnan(ret))
        ret = 0;
    return ret;
}
float vector_norm2(float v1[3], float v2[3])
{
    float ret[3] = {0};
    vector_cross_product(v1, v2, ret);
    return vector_norm(ret);
}

/*
 *  向量单位化
 */
void vector_to_unit(float v[3], float ret[3])
{
    float norm = 0;
    memcpy(ret, v, sizeof(float) * 3);
    norm = vector_norm(v);
    if (!isnan(norm) && norm != 0)
    {
        ret[0] /= norm;
        ret[1] /= norm;
        ret[2] /= norm;
    }
}

/*
 *  旋转矩阵(matrix_xyz 和 matrix_zyx 互为转置矩阵,互为逆向旋转)
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:rad
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void matrix_xyz(float roll_xyz[3], float xyz[3], float retXyz[3])
{
    float x = xyz[0], y = xyz[1], z = xyz[2];
    float A = roll_xyz[0], B = roll_xyz[1], C = roll_xyz[2];
    //这个宏用于切换坐标系方向,注意要和 matrix_zyx() 形成互为转置矩阵
#if 1
    /*
    *       [roll X]
    *   1       0       0
    *   0     cosA     sinA
    *   0    -sinA     cosA
    *
    *       [roll Y]
    *  cosB     0     -sinB
    *   0       1       0
    *  sinB     0      cosB
    *
    *       [roll Z]
    *  cosC    sinC     0
    * -sinC    cosC     0
    *   0       0       1
    *
    *                                   |x|
    *  result = [roll X][roll Y][roll Z]|y|
    *                                   |z|
    *
    *           |cB,    0,   -sB  |        |x|
    *         = |sB*sA, cA,  cB*sA|[roll Z]|y|
    *           |sB*cA, -sA, cB*cA|        |z|
    * 
    *           |cC*cB,            sC*cB,            -sB  ||x|
    *         = |cC*sB*sA - sC*cA, sC*sB*sA + cC*cA, cB*sA||x|
    *           |cC*sB*cA + sC*sA, sC*sB*cA - cC*sA, cB*cA||z|
    *
    *           |retXyz[0]|
    *         = |retXyz[1]|
    *           |retXyz[2]|
    *
    *  retXyz[*] is equal to the follow ...
    */
    retXyz[0] =
        x * cos(C) * cos(B) +
        y * sin(C) * cos(B) +
        z * (-sin(B));
    retXyz[1] =
        x * (cos(C) * sin(B) * sin(A) - sin(C) * cos(A)) +
        y * (sin(C) * sin(B) * sin(A) + cos(C) * cos(A)) +
        z * cos(B) * sin(A);
    retXyz[2] =
        x * (cos(C) * sin(B) * cos(A) + sin(C) * sin(A)) +
        y * (sin(C) * sin(B) * cos(A) - cos(C) * sin(A)) +
        z * cos(B) * cos(A);
#else
    /*
    *       [roll X]
    *   1       0       0
    *   0     cosA    -sinA
    *   0     sinA     cosA
    *
    *       [roll Y]
    *  cosB     0      sinB
    *   0       1       0
    * -sinB     0      cosB
    *
    *       [roll Z]
    *  cosC   -sinC     0
    *  sinC    cosC     0
    *   0       0       1
    *
    *                                   |x|
    *  result = [roll X][roll Y][roll Z]|y|
    *                                   |z|
    *
    *           |cB,     0,  sB    |        |x|
    *         = |sB*sA,  cA, -cB*sA|[roll Z]|y|
    *           |-sB*cA, sA, cB*cA |        |z|
    * 
    *           |cC*cB,             -sC*cB,            sB    ||x|
    *         = |cC*sB*sA + sC*cA,  -sC*sB*sA + cC*cA, -cB*sA||x|
    *           |-cC*sB*cA + sC*sA, sC*sB*cA + cC*sA,  cB*cA ||z|
    *
    *           |retXyz[0]|
    *         = |retXyz[1]|
    *           |retXyz[2]|
    *
    *  retXyz[*] is equal to the follow ...
    */
    retXyz[0] =
        x * cos(C) * cos(B) +
        y * (-sin(C) * cos(B)) +
        z * sin(B);
    retXyz[1] =
        x * (cos(C) * sin(B) * sin(A) + sin(C) * cos(A)) +
        y * (-sin(C) * sin(B) * sin(A) + cos(C) * cos(A)) +
        z * (-cos(B) * sin(A));
    retXyz[2] =
        x * (-cos(C) * sin(B) * cos(A) + sin(C) * sin(A)) +
        y * (sin(C) * sin(B) * cos(A) + cos(C) * sin(A)) +
        z * cos(B) * cos(A);
#endif
}
void matrix_zyx(float roll_xyz[3], float xyz[3], float retXyz[3])
{
    float x = xyz[0], y = xyz[1], z = xyz[2];
    float A = roll_xyz[0], B = roll_xyz[1], C = roll_xyz[2];
    //这个宏用于切换坐标系方向,注意要和 matrix_xyz() 形成互为转置矩阵
#if 1
    /*
    *       [roll Z]
    *  cosC   -sinC     0
    *  sinC    cosC     0
    *   0       0       1
    *
    *       [roll Y]
    *  cosB     0      sinB
    *   0       1       0
    * -sinB     0      cosB
    * 
    *       [roll X]
    *   1       0       0
    *   0     cosA    -sinA
    *   0     sinA     cosA
    *
    *                                   |x|
    *  result = [roll Z][roll Y][roll X]|y|
    *                                   |z|
    *
    *           |cB*cC, -sC, sB*cC|        |x|
    *         = |cB*sC, cC,  sB*sC|[roll X]|y|
    *           |-sB,   0,   cB   |        |z|
    * 
    *           |cB*cC, -cA*sC + sA*sB*cC, sA*sC + cA*sB*cC ||x|
    *         = |cB*sC, cA*cC + sA*sB*sC,  -sA*cC + cA*sB*sC||x|
    *           |-sB,   sA*cB,             cA*cB            ||z|
    *
    *           |retXyz[0]|
    *         = |retXyz[1]|
    *           |retXyz[2]|
    *
    *  retXyz[*] is equal to the follow ...
    */
    retXyz[0] =
        x * cos(B) * cos(C) +
        y * (-cos(A) * sin(C) + sin(A) * sin(B) * cos(C)) +
        z * (sin(A) * sin(C) + cos(A) * sin(B) * cos(C));
    retXyz[1] =
        x * cos(B) * sin(C) +
        y * (cos(A) * cos(C) + sin(A) * sin(B) * sin(C)) +
        z * (-sin(A) * cos(C) + cos(A) * sin(B) * sin(C));
    retXyz[2] =
        x * (-sin(B)) +
        y * sin(A) * cos(B) +
        z * cos(A) * cos(B);
#else
    /*
    *       [roll Z]
    *  cosC    sinC     0
    * -sinC    cosC     0
    *   0       0       1
    *
    *       [roll Y]
    *  cosB     0     -sinB
    *   0       1       0
    *  sinB     0      cosB
    * 
    *       [roll X]
    *   1       0       0
    *   0     cosA     sinA
    *   0    -sinA     cosA
    *
    *                                   |x|
    *  result = [roll Z][roll Y][roll X]|y|
    *                                   |z|
    *
    *           |cB*cC,  sC, -sB*cC|        |x|
    *         = |-cB*sC, cC, sB*sC |[roll X]|y|
    *           |sB,     0,  cB    |        |z|
    * 
    *           |cB*cC,  cA*sC + sA*sB*cC, sA*sC - cA*sB*cC||x|
    *         = |-cB*sC, cA*cC - sA*sB*sC, sA*cC + cA*sB*sC||x|
    *           |sB,     -sA*cB,           cA*cB           ||z|
    *
    *           |retXyz[0]|
    *         = |retXyz[1]|
    *           |retXyz[2]|
    *
    *  retXyz[*] is equal to the follow ...
    */
    retXyz[0] =
        x * cos(B) * cos(C) +
        y * (cos(A) * sin(C) + sin(A) * sin(B) * cos(C)) +
        z * (sin(A) * sin(C) - cos(A) * sin(B) * cos(C));
    retXyz[1] =
        x * (-cos(B) * sin(C)) +
        y * (cos(A) * cos(C) - sin(A) * sin(B) * sin(C)) +
        z * (sin(A) * cos(C) + cos(A) * sin(B) * sin(C));
    retXyz[2] =
        x * sin(B) +
        y * (-sin(A) * cos(B)) +
        z * cos(A) * cos(B);
#endif
}
// roll_xyz使用单位: 度
void matrix_xyz2(float roll_xyz[3], float xyz[3], float retXyz[3])
{
    float _roll_xyz[3];
    _roll_xyz[0] = roll_xyz[0] * MATH_PI / 180;
    _roll_xyz[1] = roll_xyz[1] * MATH_PI / 180;
    _roll_xyz[2] = roll_xyz[2] * MATH_PI / 180;
    matrix_xyz(_roll_xyz, xyz, retXyz);
}
void matrix_zyx2(float roll_xyz[3], float xyz[3], float retXyz[3])
{
    float _roll_xyz[3];
    _roll_xyz[0] = roll_xyz[0] * MATH_PI / 180;
    _roll_xyz[1] = roll_xyz[1] * MATH_PI / 180;
    _roll_xyz[2] = roll_xyz[2] * MATH_PI / 180;
    matrix_zyx(_roll_xyz, xyz, retXyz);
}

/*
 *  矩阵运算: 透视矩阵点乘三维坐标,然后除以z(透视除法),返回投影坐标[-ar, ar]U[-1, 1]
 * 
 *  参数:
 *      openAngle: 相机开角(单位:rad,范围:(0,pi))
 *      xyz[3]: 要计算的空间坐标
 *      ar: 相机的屏幕的宽高比
 *      nearZ: 相机近端距离
 *      farZ: 相机远端距离
 *      retXY: 计算结果,一个二维平面坐标(注意其坐标原点是屏幕中心)
 *      retDepth: 计算结果,深度值(远离屏幕的距离,单位:点)
 * 
 *  返回: false/不再相框内  true/在相框内
 */
bool projection(
    float openAngle,
    float xyz[3],
    float ar,
    int nearZ,
    int farZ,
    float *retXY,
    float *retDepth)
{
    float hMax, hMin, wMax, wMin;
    float retX, retY, retZ;

    //快速检查
    if (openAngle >= 360 || openAngle < 1)
        return false;
    if (ar <= 0 ||
        xyz == NULL ||
        nearZ >= farZ ||
        xyz[0] < nearZ ||
        xyz[0] > farZ)
        return false;

    //度转rad
    openAngle = openAngle * MATH_PI / 180;

    //屏幕高、宽范围(这里是假设屏幕高为2时的数值)
    hMax = 1;
    hMin = -1;
    wMax = ar;
    wMin = -ar;

    /*                          [project matrix]
    *
    *   1/ar/tan(a/2)           0               0               0   
    *       0               1/tan(a/2)          0               0
    *       0                   0     ((-nZ)-fZ)/(nZ-fZ)  2*fZ*nZ/(nZ-fZ)
    *       0                   0               1               0
    *
    *   ar: camera width/height
    *   a: camera open angle
    *   nZ: camera near Z
    *   fZ: camera far Z
    *
    *                   |x|               |retX|
    *   [project matrix]|y| and then /z = |retY|
    *                   |z|               |retZ|
    *                   |1|               | 1  |
    *
    *   output point request: retX in the range of (-ar, ar)
    *                         retY in the range of (-1, 1)
    *                         retZ in the range of (-1, 1)
    */
    retX = xyz[1] / ar / tan(openAngle / 2) / xyz[0];
    retY = xyz[2] / tan(openAngle / 2) / xyz[0];
    retZ = ((-nearZ) - farZ) / (nearZ - farZ) + 2 * farZ * nearZ / (nearZ - farZ) / xyz[0];

    //返回二维坐标
    if (retXY)
    {
        retXY[0] = retX;
        retXY[1] = retY;
    }
    //深度
    if (retDepth)
        *retDepth = xyz[0] - nearZ;
    //是否在相框范围内
    if (wMax > retX && retX > wMin &&
        hMax > retY && retY > hMin &&
        1 > retZ && retZ > -1)
    {
        return true;
    }
    return false;
}
