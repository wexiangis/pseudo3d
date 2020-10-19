#include <math.h>
#include "i2c_transfer.h"

#define HMC5883_ID 0x3C //定义器件在IIC总线中的从地址

static char hmc5883_start_flag = 0;
static float hmc5883_result = 0;

//获取罗盘角度(rad:[-pi, pi])
float hmc5883_get(void)
{
    unsigned char data[6] = {0};
    // short z;
    short x, y;

    // init
    if (!hmc5883_start_flag)
    {
        hmc5883_start_flag = 1;
        data[0] = 0x00;
        i2c_default_rw(HMC5883_ID, 0x02, 1, data, 1);
    }
    // get x, y, z data
    if (i2c_default_rw(HMC5883_ID, 0x03, 6, data, 0) == 0)
    {
        x = (data[0] << 8) | data[1];
        y = (data[4] << 8) | data[5];
        // z = (data[2] << 8) | data[3];
        hmc5883_result = atan2((float)y, (float)x);
    }

    return hmc5883_result;
}
