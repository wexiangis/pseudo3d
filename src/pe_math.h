#ifndef _PE_MATK_H_
#define _PE_MATK_H_

/*
 *  valG: 陀螺仪xyz轴输出rad/s
 *  valA: 加速度xyz轴输出g
 *  pry: 输出绕xyz轴角度(单位:rad)
 *  intervalMs: 采样间隔(单位:ms)
 */
void quaternion(float *valG, float *valA, float *pry, int intervalMs);

/*
 *  把空间坐标point[3]转换为物体自身坐标系
 *  参数:
 *      raxyz[3] : 绕X/Y/Z轴的转角(rad: 0~2pi)
 *      point[3] : 要修正的空间向量的坐标,输出值回写到这里面
 */
void matrix_xyz(float raxyz[3], float point[3]);

/*
 *  把物体自身坐标point[3]转换为空间坐标
 *  参数:
 *      raxyz[3] : 绕X/Y/Z轴的转角(rad: 0~2pi)
 *      point[3] : 要修正的空间向量的坐标,输出值回写到这里面
 */
void matrix_zyx(float raxyz[3], float point[3]);

#endif
