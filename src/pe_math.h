#ifndef _PE_MATK_H_
#define _PE_MATK_H_

#include <stdbool.h>

/*
 *  quaternion解算
 *  参数:
 *      valG: 陀螺仪xyz轴输出,单位:deg/s (必要参数)
 *      valA: 加速度xyz轴输出,单位:g  (可以置NULL,等于纯陀螺仪计算姿态)
 *      pry: 输出绕xyz轴角度,单位:rad (可以置NULL)
 *      intervalMs: 采样间隔,单位:ms (必要参数)
 */
void quat_pry(float *valG, float *valA, float *pry, int intervalMs);

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
 */
void quat_xyz(float roll_xyz[3], float xyz[3]);
void quat_zyx(float roll_xyz[3], float xyz[3]);

/*
 *  使用现有四元数进行旋转矩阵运算
 */
void quat_matrix_xyz(float quat[4], float xyz[3]);
void quat_matrix_zyx(float quat[4], float xyz[3]);

/*
 *  旋转矩阵
 *  参数:
 *      raxyz[3] : 绕三轴转角(rad: 0~2pi)
 *      point[3] : 要修正的空间向量的坐标,输出值回写到这里面
 */
void matrix_xyz(float raxyz[3], float point[3]);
void matrix_zyx(float raxyz[3], float point[3]);

#endif
