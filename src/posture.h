/*
 *  根据MPU6050数据计算姿态
 */
#ifndef _POSTURE_H_
#define _POSTURE_H_

#include <pthread.h>

typedef struct
{
    //线程及其运行标志
    pthread_t th;
    short flagRun;
    //采样周期
    int intervalMs;

    //绕轴角速度(单位:deg/s)
    float gyrXYZ[3];
    //三轴合受力(单位:g)
    float accXYZ[3];
    //除重力以外的合受力(单位:g)
    //也就是说上面accXYZ[3]减去下面的向量就是仅重力的受力
    float accForce[3];

    //角速度累加得到的角度值(相对自身坐标,rad:[-pi, pi])
    float gyrRollXYZ[3];
    float gyrRollXYZ2[3]; //degree mode
    //重力加速度得到的角度值(相对空间坐标,rad:[-pi, pi])
    float accRollXYZ[3];
    //最终输出角度值(相对空间坐标,rad:[-pi, pi])
    float rollXYZ[3];
    float rollXYZ2[3]; //器件自身输出
    //偏航角较正
    float rollZErr;

    //空间坐标系下的横纵向g值(单位:g)
    float gX, gY, gZ, gXYZ;
    //correct gX/gY (单位:g)
    float gXErr, gYErr, gZErr;
    //accel in horizomtal X/Y (unit:m/ss)
    float aX, aY, aZ;
    //空间坐标系下的横纵向速度(单位:m/s)
    float speX, speY, speZ;
    //空间坐标系下的横纵向偏移距离(单位:m)
    float movX, movY, movZ;

    //罗盘原始数据
    float compassXYZ[3];
    //水平方向(单位:rad)
    float dir;
    //温度(原始数值)
    float temper;
} PostureStruct;

/*
 *  初始化
 * 
 *  intervalMs: 采样间隔, 越小误差积累越小, 建议值:10(推荐),20,25,50
 */
PostureStruct *pe_init(int intervalMs);
void pe_exit(PostureStruct **ps);

//复位(重置计算值)
void pe_reset(PostureStruct *ps);

//获取罗盘角度(rad:[-pi, pi])
float pe_dir(void);

#endif

