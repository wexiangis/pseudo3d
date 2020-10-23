
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

//q30格式,long转float时的除数.
#define q30 1073741824.0f

//陀螺仪方向设置
static signed char gyro_orientation[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1};

//返回值: 0/正常
int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3)
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        return 0;
    }
    else
        return 1;
}

//方向转换
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7; // error
    return b;
}

//陀螺仪方向控制
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

/*
 *  传感器精度: gyro/2000 accel/2g
 *  Hz: 设置陀螺仪和角速度计采样频率,推荐值: 50, 25, 20, 10
 *  test: 是否启用自测(启动会慢很多)
 *  返回值: 0/正常
 */
int mpu6050_init(unsigned short Hz, char test)
{
    if (mpu_init() == 0)
    {
        //设置用到的传感器
#ifdef MPU9250
        if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) != 0)
#else
        if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0)
#endif
            return 1;
        //设置FIFO
        if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0)
            return 2;
        //设置陀螺仪和角速度计采样率
        if (mpu_set_sample_rate(Hz) != 0)
            return 3;
#ifdef MPU9250
        //设置罗盘采样率(最大100但没必要)
        if (mpu_set_compass_sample_rate(10) != 0)
            return 4;
#endif
        //加载dmp固件
        if (dmp_load_motion_driver_firmware() != 0)
            return 5;
        //设置陀螺仪方向
        if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)) != 0)
            return 6;
        //设置dmp功能
        if (dmp_enable_feature(
                DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
                DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL) != 0)
            return 7;
        //设置DMP输出速率(最大不超过200Hz)
        if (dmp_set_fifo_rate(Hz) != 0)
            return 8;
        if (test)
        {
            //自检
            if (run_self_test() != 0)
                return 9;
        }
        //使能DMP
        if (mpu_set_dmp_state(1) != 0)
            return 10;
    }
    else
        return 99;
    return 0;
}

/*
 *  得到dmp处理后的数据
 *  pry: 欧拉角 pitch, roll, yaw, 单位rad
 *  gyro: 陀螺仪原始数据
 *  accel: 重力加速度原始数据
 *  返回值: 0/正常
 */
int mpu6050_angle(double *pry, short *gyro, short *accel)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    unsigned long sensor_timestamp;
    short sensors;
    unsigned char more;
    long quat[4];
    if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more) != 0)
        return 1;
    /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
    if (sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / q30; //q30格式转换为浮点数
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;
        //计算得到俯仰角/横滚角/航向角
        pry[0] = asin(-2 * q1 * q3 + 2 * q0 * q2);                                      // pitch
        pry[1] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1);      // roll
        pry[2] = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3); // yaw
    }
    else
        return 2;
    return 0;
}

/*
 *  取罗盘数据
 */
int mpu6050_compass(short *compass)
{
#ifndef MPU9250
    return 0;
#else
    unsigned long sensor_timestamp;
    return mpu_get_compass_reg(compass, &sensor_timestamp);
#endif
}

/*
 *  取温度数据
 */
int mpu6050_temper(long *temper)
{
    unsigned long sensor_timestamp;
    return mpu_get_temperature(temper, &sensor_timestamp);
}
