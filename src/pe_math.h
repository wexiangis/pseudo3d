#ifndef _PE_MATH_H_
#define _PE_MATH_H_

#include <stdbool.h>

/*
 *  quaternion解算
 *  参数:
 *      quat_err[10]: 四元数和误差积累数组,初始值用 {1,0,0,0,0,0,0,0,0,0} (必要参数)
 *      valG[3]: 陀螺仪xyz轴输出,单位:deg/s (必要参数)
 *      valA[3]: 加速度xyz轴输出,单位:g  (可以置NULL,等于纯陀螺仪计算姿态)
 *      pry[3]: 输出绕xyz轴角度,单位:rad (可以置NULL)
 *      intervalMs: 采样间隔,单位:ms (必要参数)
 */
void quat_pry(float quat_err[10], float valG[3], float valA[3], float pry[3], int intervalMs, float miscRate);

// 四元数角增量(龙格塔微分方程)
void quat_diff(float q[4], float roll_xyz[3]);
// roll_xyz使用单位: 度
void quat_diff2(float q[4], float roll_xyz[3]);

// 四元数乘法
void quat_multiply(float q1[4], float q2[4], float ret[4]);

// 欧拉角转四元数(zyx顺序)
void pry_to_quat(float pry[3], float q[4]);
// pry使用单位: 度
void pry_to_quat2(float pry[3], float q[4]);

// 四元数转欧拉角
void quat_to_pry(float q[4], float pry[3]);
// pry使用单位: 度
void quat_to_pry2(float q[4], float pry[3]);

/*
 *  四元数方式旋转和逆旋转
 *  参数:
 *      quat[4]: 使用已有的四元数(可置NULL), 将不使用 roll_vector 和 roll_rad
 *      roll_vector[3]: 要绕转的空间向量,右手旋转,大拇指向量方向
 *      roll_rad: 旋转角度,单位:rad
 *      vector[3]: 被旋转的向量,输出结果覆写到此
 *      T: 转置
 */
void quat_roll(float quat[4], float roll_vector[3], float roll_rad, float vector[3], bool T);

/*
 *  四元数依次三轴旋转
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:rad
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void quat_xyz(float roll_xyz[3], float xyz[3], float retXyz[3]);
void quat_zyx(float roll_xyz[3], float xyz[3], float retXyz[3]);

/*
 *  使用现有四元数进行旋转矩阵运算
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:rad
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void quat_matrix_xyz(float quat[4], float xyz[3], float retXyz[3]); // 待验证
void quat_matrix_zyx(float quat[4], float xyz[3], float retXyz[3]);

/*
 *  向量叉乘, v1 x v2 = ret
 */
void vector_cross_product(float v1[3], float v2[3], float ret[3]);

/*
 *  向量取模
 */
float vector_norm(float v[3]);
float vector_norm2(float v1[3], float v2[3]);

/*
 *  向量单位化
 */
void vector_to_unit(float v[3], float ret[3]);

/*
 *  旋转矩阵(matrix_xyz 和 matrix_zyx 互为转置矩阵,互为逆向旋转)
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:rad
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void matrix_xyz(float roll_xyz[3], float xyz[3], float retXyz[3]);
void matrix_zyx(float roll_xyz[3], float xyz[3], float retXyz[3]);
// roll_xyz使用单位: 度
void matrix_xyz2(float roll_xyz[3], float xyz[3], float retXyz[3]);
void matrix_zyx2(float roll_xyz[3], float xyz[3], float retXyz[3]);

/*
 *  透视矩阵点乘三维坐标,然后除以z(透视除法),返回投影坐标[-ar, ar]U[-1, 1]
 * 
 *  参数:
 *      openAngle: 相机开角(单位:度,范围:[1,359])
 *      xyz[3]: 要计算的空间坐标
 *      ar: 相机的屏幕的宽高比
 *      nearZ: 相机近端距离
 *      farZ: 相机远端距离
 *      retXY: 计算结果,一个二维平面坐标(注意其坐标原点是屏幕中心)
 *      retDepth: 计算结果,深度值(远离屏幕的距离,单位:点)
 * 
 *  返回: 0/不再相框内  1/在相框内
 */
bool projection(
    float openAngle,
    float xyz[3],
    float ar,
    int nearZ,
    int farZ,
    float *retXY,
    float *retDepth);

#endif
