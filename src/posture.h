/*
 *  根据MPU6050数据计算姿态
 */
#ifndef _POSTURE_H_
#define _POSTURE_H_

#include <stdbool.h>
#include <pthread.h>

typedef struct
{
    void *obj;
    void (*callback)(void *);
    pthread_t th;
    short flagRun;
    int intervalMs;

    //使用四元数法时用
    float quat_err[10];  //计算姿态时用
    float quat_err2[10]; //计算陀螺仪姿态时用

    //绕轴角速度(单位:deg/s)
    float gyrXYZ[3];
    float gyrXYZErr[3];
    //三轴合受力(单位:g)
    float accXYZ[3];
    float accXYZ2;

    //除重力以外的合受力(单位:g)
    float gForce[3];

    //角速度累加得到的角度值(相对自身坐标,rad:[-pi, pi])
    float gyrRollXYZ[3];
    //重力加速度得到的角度值(相对空间坐标,rad:[-pi, pi])
    float accRollXYZ[3];
    //最终输出角度值(相对空间坐标,rad:[-pi, pi])
    float rollXYZ[3];
    //偏航角较正
    float rollZErr;

    //空间坐标系下的三轴g值(单位:g)
    float gXYZ[3];
    float gXYZErr[3];
    //空间坐标系下的三轴加速度(单位:m/ss)
    float aXYZ[3];
    //空间坐标系下的三轴速度(单位:m/s)
    float speXYZ[3];
    //空间坐标系下的三轴位移(单位:m)
    float movXYZ[3];

    //罗盘原始数据
    float compassXYZ[3];
    //水平方向(单位:rad)
    float dir;
    //温度(原始数值)
    float temper;

    //角速度变化率(角速度向量的模)
    float gyrRate;
    //加速度变化率(前后加速度向量的叉乘,再取模)
    float accRate;
    //综合变化率
    float miscRate;
    //静止标志
    bool quiet;

} PostureStruct;

/*
 *  初始化
 * 
 *  intervalMs: 采样间隔, 越小误差积累越小, 建议值:10(推荐),20,25,50
 */
PostureStruct *pe_init(int intervalMs, void *obj, void (*callback)(void *));
void pe_exit(PostureStruct **ps);

//复位(重置计算值)
void pe_reset(PostureStruct *ps);

//获取罗盘角度(rad:[-pi, pi])
float pe_dir(void);

#endif
