#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MATH_PI 3.14159265358979323846

/*
 *  quaternion解算
 *  参数:
 *      valG: 陀螺仪xyz轴输出,单位:deg/s (必要参数)
 *      valA: 加速度xyz轴输出,单位:g  (可以置NULL,等于纯陀螺仪计算姿态)
 *      pry: 输出绕xyz轴角度,单位:rad (可以置NULL)
 *      intervalMs: 采样间隔,单位:ms (必要参数)
 */
void quat_pry(float *valG, float *valA, float *pry, int intervalMs)
{
    float Kp = 2.0f;
    float Ki = 0.001f;
    // 四元数的元素，代表估计方向
    static float qBak[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    // 按比例缩小积分误差
    static float eIntBak[3] = {0.0f, 0.0f, 0.0f};
    // 时间间隔一半, 后面 pi/180 用于 deg/s 转 rad/s
    float halfT = (float)intervalMs / 2 / 1000;
    float q[4];
    float eInt[3];
    float norm;
    float ax, ay, az;
    float gx, gy, gz;
    float vx, vy, vz;
    float ex, ey, ez;
    if (!valG)
        return;
    // stack out
    memcpy(q, qBak, sizeof(float) * 4);
    memcpy(eInt, eIntBak, sizeof(float) * 3);
    // 估计重力的方向(vx, vy, vz)
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
            // 动态参数,当重力失真(自由落体/超重)时减少对加速度计依赖
            if (norm < 1.0f)
            {
                norm = pow(norm, 5);
                Kp *= norm;
                Ki *= norm;
            }
            else if (norm > 1.0 && norm < 2.0f)
            {
                norm = pow(2.0f - norm, 5);
                Kp *= norm;
                Ki *= norm;
            }
            else if (norm >= 2.0f)
                Kp = Ki = 0.0f;
        }
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
    // 调整后的陀螺仪测量
    gx = valG[0] * MATH_PI / 180 + Kp * ex + eInt[0];
    gy = valG[1] * MATH_PI / 180 + Kp * ey + eInt[1];
    gz = valG[2] * MATH_PI / 180 + Kp * ez + eInt[2];
    // 四元数微分方程
    q[0] += (-q[1] * gx - q[2] * gy - q[3] * gz) * halfT;
    q[1] += (q[0] * gx + q[2] * gz - q[3] * gy) * halfT;
    q[2] += (q[0] * gy - q[1] * gz + q[3] * gx) * halfT;
    q[3] += (q[0] * gz + q[1] * gy - q[2] * gx) * halfT;
    // 正常化四元,单位化
    norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (isnan(norm))
    {
        // printf(" isnan Q2 \r\n");
        return;
    }
    q[0] = q[0] / norm;
    q[1] = q[1] / norm;
    q[2] = q[2] / norm;
    q[3] = q[3] / norm;
    // pry
    if (pry)
    {
        pry[0] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]);
        pry[1] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1);
        pry[2] = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    }
    // stack in
    memcpy(qBak, q, sizeof(float) * 4);
    memcpy(eIntBak, eInt, sizeof(float) * 3);
    // printf(" Q %.4f %.4f %.4f %.4f // %.4f %.4f %.4f \r\n",
    //     q[0], q[1], q[2], q[3], eInt[0], eInt[1], eInt[2]);
}

// 四元数乘法
static void _quat_multiply(float q1[4], float q2[4], float ret[4])
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

// 四元数转欧拉角
static void _quat_to_pry(float q[4], float pry[3])
{
    pry[0] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]);
    pry[1] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1);
    pry[2] = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
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
    float *q = quat;
    float _q[4], qT[4];
    float rv[3];
    float v[4], ret[4];
    float norm;

    if (!q)
    {
        //对旋转轴进行单位向量处理(否则旋转后会附带缩放效果)
        memcpy(rv, roll_vector, sizeof(float) * 3);
        norm = sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
        if (!isnan(norm))
        {
            rv[0] /= norm;
            rv[1] /= norm;
            rv[2] /= norm;
        }
        //
        q = _q;
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
        _quat_multiply(qT, v, ret);
        _quat_multiply(ret, q, ret);
    }
    else
    {
        _quat_multiply(q, v, ret);
        _quat_multiply(ret, qT, ret);
    }

    // norm = sqrt(ret[1] * ret[1] + ret[2] * ret[2] + ret[3] * ret[3]);
    // if (!isnan(norm))
    // {
    //     ret[1] /= norm;
    //     ret[2] /= norm;
    //     ret[3] /= norm;

    //     norm = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    //     if (!isnan(norm))
    //     {
    //         ret[1] *= norm;
    //         ret[2] *= norm;
    //         ret[3] *= norm;
    //     }
    // }

    memcpy(vector, &ret[1], sizeof(float) * 3);
}

static void _quat_roll_xyz(float roll_xyz[3], float vector[3], bool zyx)
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
    v[1] = vector[0];
    v[2] = vector[1];
    v[3] = vector[2];

    if (zyx)
    {
        _quat_multiply(qz, qy, ret);
        _quat_multiply(ret, qx, ret);
        _quat_multiply(ret, v, ret);
        _quat_multiply(ret, qxT, ret);
        _quat_multiply(ret, qyT, ret);
        _quat_multiply(ret, qzT, ret);

        // _quat_multiply(qxT, qyT, ret);
        // _quat_multiply(ret, qzT, ret);
        // _quat_multiply(ret, v, ret);
        // _quat_multiply(ret, qz, ret);
        // _quat_multiply(ret, qy, ret);
        // _quat_multiply(ret, qx, ret);
    }
    else
    {
        _quat_multiply(qx, qy, ret);
        _quat_multiply(ret, qz, ret);
        _quat_multiply(ret, v, ret);
        _quat_multiply(ret, qzT, ret);
        _quat_multiply(ret, qyT, ret);
        _quat_multiply(ret, qxT, ret);
    }

    memcpy(vector, &ret[1], sizeof(float) * 3);
}

/*
 *  四元数依次三轴旋转
 */
void quat_xyz(float roll_xyz[3], float xyz[3])
{
    _quat_roll_xyz(roll_xyz, xyz, false);
}
void quat_zyx(float roll_xyz[3], float xyz[3])
{
    _quat_roll_xyz(roll_xyz, xyz, true);
}

/*
 *  使用现有四元数进行旋转矩阵运算
 */
void quat_matrix_xyz(float quat[4], float xyz[3])
{
    float q0 = quat[0];
    float q1 = quat[1];
    float q2 = quat[2];
    float q3 = quat[3];
    float ret[3];
    // float norm;

    ret[0] =
        xyz[0] * (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) +
        xyz[1] * 2 * (q1 * q2 - q0 * q3) +
        xyz[2] * 2 * (q1 * q3 + q0 * q2);
    ret[1] =
        xyz[0] * 2 * (q1 * q2 + q0 * q3) +
        xyz[1] * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) +
        xyz[2] * 2 * (q2 * q3 + q0 * q1);
    ret[2] =
        xyz[0] * 2 * (q1 * q3 - q0 * q2) +
        xyz[1] * 2 * (q1 * q2 + q0 * q3) +
        xyz[2] * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    
    // norm = sqrt(ret[0] * ret[0] + ret[1] * ret[1] + ret[2] * ret[2]);
    // if (!isnan(norm))
    // {
    //     ret[0] /= norm;
    //     ret[1] /= norm;
    //     ret[2] /= norm;

    //     norm = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
    //     if (!isnan(norm))
    //     {
    //         ret[0] *= norm;
    //         ret[1] *= norm;
    //         ret[2] *= norm;
    //     }
    // }

    memcpy(xyz, ret, sizeof(float) * 3);
}
void quat_matrix_zyx(float quat[4], float xyz[3])
{
    float q0 = quat[0];
    float q1 = quat[1];
    float q2 = quat[2];
    float q3 = quat[3];
    float ret[3];
    // float norm;

    ret[0] =
        xyz[0] * (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) +
        xyz[1] * 2 * (q1 * q2 + q0 * q3) +
        xyz[2] * 2 * (q1 * q3 - q0 * q2);
    ret[1] =
        xyz[0] * 2 * (q1 * q2 - q0 * q3) +
        xyz[1] * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) +
        xyz[2] * 2 * (q2 * q3 + q0 * q1);
    ret[2] =
        xyz[0] * 2 * (q1 * q3 + q0 * q2) +
        xyz[1] * 2 * (q2 * q3 + q0 * q1) +
        xyz[2] * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    
    // norm = sqrt(ret[0] * ret[0] + ret[1] * ret[1] + ret[2] * ret[2]);
    // if (!isnan(norm))
    // {
    //     ret[0] /= norm;
    //     ret[1] /= norm;
    //     ret[2] /= norm;

    //     norm = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
    //     if (!isnan(norm))
    //     {
    //         ret[0] *= norm;
    //         ret[1] *= norm;
    //         ret[2] *= norm;
    //     }
    // }

    memcpy(xyz, ret, sizeof(float) * 3);
}

/*
 *  把空间坐标point[3]转换为物体自身坐标系
 *  参数:
 *      raxyz[3] : 绕X/Y/Z轴的转角(rad: 0~2pi)
 *      point[3] : 要修正的空间向量的坐标,输出值回写到这里面
 */
void matrix_xyz(float raxyz[3], float point[3])
{
    float x = point[0], y = point[1], z = point[2];
    float A = raxyz[0], B = raxyz[1], C = raxyz[2];
    //这个宏用于切换坐标系方向,注意要和matrix_xyz()形成互为转置矩阵
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
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */
    point[0] =
        x * cos(C) * cos(B) +
        y * sin(C) * cos(B) +
        z * (-sin(B));
    point[1] =
        x * (cos(C) * sin(B) * sin(A) - sin(C) * cos(A)) +
        y * (sin(C) * sin(B) * sin(A) + cos(C) * cos(A)) +
        z * cos(B) * sin(A);
    point[2] =
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
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */
    point[0] =
        x * cos(C) * cos(B) +
        y * (-sin(C) * cos(B)) +
        z * sin(B);
    point[1] =
        x * (cos(C) * sin(B) * sin(A) + sin(C) * cos(A)) +
        y * (-sin(C) * sin(B) * sin(A) + cos(C) * cos(A)) +
        z * (-cos(B) * sin(A));
    point[2] =
        x * (-cos(C) * sin(B) * cos(A) + sin(C) * sin(A)) +
        y * (sin(C) * sin(B) * cos(A) + cos(C) * sin(A)) +
        z * cos(B) * cos(A);
#endif
}

/*
 *  把物体自身坐标point[3]转换为空间坐标
 *  参数:
 *      raxyz[3] : 绕X/Y/Z轴的转角(rad: 0~2pi)
 *      point[3] : 要修正的空间向量的坐标,输出值回写到这里面
 */
void matrix_zyx(float raxyz[3], float point[3])
{
    float x = point[0], y = point[1], z = point[2];
    float A = raxyz[0], B = raxyz[1], C = raxyz[2];
    //这个宏用于切换坐标系方向,注意要和matrix_xyz()形成互为转置矩阵
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
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */
    point[0] =
        x * cos(B) * cos(C) +
        y * (-cos(A) * sin(C) + sin(A) * sin(B) * cos(C)) +
        z * (sin(A) * sin(C) + cos(A) * sin(B) * cos(C));
    point[1] =
        x * cos(B) * sin(C) +
        y * (cos(A) * cos(C) + sin(A) * sin(B) * sin(C)) +
        z * (-sin(A) * cos(C) + cos(A) * sin(B) * sin(C));
    point[2] =
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
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */
    point[0] =
        x * cos(B) * cos(C) +
        y * (cos(A) * sin(C) + sin(A) * sin(B) * cos(C)) +
        z * (sin(A) * sin(C) - cos(A) * sin(B) * cos(C));
    point[1] =
        x * (-cos(B) * sin(C)) +
        y * (cos(A) * cos(C) - sin(A) * sin(B) * sin(C)) +
        z * (sin(A) * cos(C) + cos(A) * sin(B) * sin(C));
    point[2] =
        x * sin(B) +
        y * (-sin(A) * cos(B)) +
        z * cos(A) * cos(B);
#endif
}
