
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#ifdef MPU9250
#include "i2c_transfer.h"
#define i2c_write(slave_addr, reg_addr, length, data) i2c_default_rw(slave_addr, reg_addr, length, data, 1)
#define i2c_read(slave_addr, reg_addr, length, data) i2c_default_rw(slave_addr, reg_addr, length, data, 0)
#endif

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

//返回值: 0/正常
int mpu6050_init(unsigned short Hz, char test)
{
    int res = 0;
    // unsigned char data[1];

    //默认精度: gyro/2000 accel/2g
    if (mpu_init() == 0)
    {
        // data[0] = 0x02;
        // i2c_write(0x68, 0x37, 1, data);//turn on Bypass Mode
#ifdef MPU6050
        res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); //设置所需要的传感器
#else
        res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); //设置所需要的传感器
#endif
        if (res)
            return 1;
        res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); //设置FIFO
        if (res)
            return 2;
        res = mpu_set_sample_rate(Hz); //设置采样率
        if (res)
            return 3;
        res = dmp_load_motion_driver_firmware(); //加载dmp固件
        if (res)
            return 4;
        res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)); //设置陀螺仪方向
        if (res)
            return 5;
        res = dmp_enable_feature(
            DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | //设置dmp功能
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
        if (res)
            return 6;
        res = dmp_set_fifo_rate(Hz); //设置DMP输出速率(最大不超过200Hz)
        if (res)
            return 7;
        if(test) {
            res = run_self_test(); //自检
            if (res)
                return 8;
        }
        res = mpu_set_dmp_state(1); //使能DMP
        if (res)
            return 9;
    }
    else
        return 10;
    return 0;
}

/*
 *  得到dmp处理后的数据
 *  pry:
 *      pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
 *      roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
 *      yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
 *  返回值: 0/正常
 */
int mpu6050_get(double *pry, short *gyro, short *accel, short *dir)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    unsigned long sensor_timestamp;
    short sensors;
    unsigned char more;
    long quat[4];
#ifdef MPU9250
    short _dir[3];
#endif
    if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
        return 1;
    /* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
#ifdef MPU9250
    if(!dir)
        mpu_get_compass_reg(dir, &sensor_timestamp);
    else
        mpu_get_compass_reg(_dir, &sensor_timestamp);
#endif
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
