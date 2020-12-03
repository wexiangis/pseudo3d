#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*
 *  quaternion解算
 *  参数:
 *      valG: 陀螺仪xyz轴输出rad/s
 *      valA: 加速度xyz轴输出g
 *      pry: 输出绕xyz轴角度(单位:rad)
 *      intervalMs: 采样间隔(单位:ms)
 */
void quat(float *valG, float *valA, float *pry, int intervalMs)
{
    float Kp = 200.0f; // 100.0f;
    float Ki = 0.002f; // 0.002f;
    // 四元数的元素，代表估计方向
    static float qBak[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    // 按比例缩小积分误差
    static float eIntBak[3] = {0.0f, 0.0f, 0.0f};
    // 我也不知道为什么要除以 60 ...
    float halfT = (float)intervalMs / 2 / 1000 / 60;
    float q[4];
    float eInt[3];
    float norm;
    float ax, ay, az;
    float gx, gy, gz;
    float vx, vy, vz;
    float ex, ey, ez;
    // stack in
    memcpy(q, qBak, sizeof(float) * 4);
    memcpy(eInt, eIntBak, sizeof(float) * 3);
    // 测量正常化,单位化
    norm = sqrt(valA[0] * valA[0] + valA[1] * valA[1] + valA[2] * valA[2]);
    if (isnan(norm))
    {
        // printf(" isnan Q1 \r\n");
        return;
    }
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
        norm = pow(norm - 1.0f, 5);
        Kp *= norm;
        Ki *= norm;
    }
    else if (norm >= 2.0f)
        Kp = Ki = 0.0f;
    // 估计重力的方向
    vx = 2 * (q[1] * q[3] - q[0] * q[2]);
    vy = 2 * (q[0] * q[1] + q[2] * q[3]);
    vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    // 积分误差比例积分增益
    eInt[0] = eInt[0] + ex * Ki;
    eInt[1] = eInt[1] + ey * Ki;
    eInt[2] = eInt[2] + ez * Ki;
    // 调整后的陀螺仪测量
    gx = valG[0] + Kp * ex + eInt[0];
    gy = valG[1] + Kp * ey + eInt[1];
    gz = valG[2] + Kp * ez + eInt[2];
    // 整合四元数率和正常化
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
    pry[0] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]);
    pry[1] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1);
    pry[2] = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    // stack out
    memcpy(qBak, q, sizeof(float) * 4);
    memcpy(eIntBak, eInt, sizeof(float) * 3);
    // printf(" Q %.4f %.4f %.4f %.4f // %.4f %.4f %.4f \r\n",
    //     q[0], q[1], q[2], q[3], eInt[0], eInt[1], eInt[2]);
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
    //这个宏用于切换坐标系方向,注意要和matrix_xyz()形成互为逆矩阵
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
    //这个宏用于切换坐标系方向,注意要和matrix_xyz()形成互为逆矩阵
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
