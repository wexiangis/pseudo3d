#include "i2c_transfer.h"

#define STATUS_F_STATUS 0x00
#define OUT_X_MSB  0x01
#define OUT_X_LSB  0x02
#define OUT_Y_MSB  0x03
#define OUT_Y_LSB  0x04
#define OUT_Z_MSB  0x05
#define OUT_Z_LSB  0x06
#define F_SETUP    0x09
#define TRIG_CFG   0x0A
#define SYSMOD     0x0B
#define INT_SOURCE 0x0C
#define WHO_AM_I   0x0D
#define XYZ_DATA_CFG     0x0E
#define HP_FILTER_CUTOFF 0x0F
#define PL_STATUS  0x10
#define PL_CFG     0x11
#define PL_COUNT   0x12
#define PL_BF_ZCOMP 0x13
#define P_L_THS_REG 0x14
#define FF_MT_CFG   0x15
#define FF_MT_SRC   0x16
#define FF_MT_THS   0x17
#define FF_MT_COUNT 0x18
#define TRANSIENT_CFG 0x1D
#define TRANSIENT_SRC 0x1E
#define TRANSIENT_THS 0x1F
#define TRANSIENT_COUNT 0x20
#define PULSE_CFG  0x21
#define PULSE_SRC  0x22
#define PULSE_THSX 0x23
#define PULSE_THSY 0x24
#define PULSE_THSZ 0x25
#define PULSE_TMLT 0x26
#define PULSE_LTCY 0x27
#define PULSE_WIND 0x28
#define ASLP_COUNT 0x29
#define CTRL_REG1  0x2A
#define CTRL_REG2  0x2B
#define CTRL_REG3  0x2C
#define CTRL_REG4  0x2D
#define CTRL_REG5  0x2E
#define OFF_X 0x2F
#define OFF_Y 0x30
#define OFF_Z 0x31

#define MMA8451_ID 0x1C

void mma8451_init(void)
{
    unsigned char data[1];
    // output data rate 800Hz, active mode
    data[0] = 0x01;
    i2c_default_rw(MMA8451_ID, CTRL_REG1, 1, data, 1);
    // high resoultion
    data[0] = 0x02;
    i2c_default_rw(MMA8451_ID, CTRL_REG2, 1, data, 1);
    // enable high-pass ouput, range 2g
    data[0] = 0x10;
    i2c_default_rw(MMA8451_ID, XYZ_DATA_CFG, 1, data, 1);
    // disable low-pass filter
    data[0] = 0x00;
    i2c_default_rw(MMA8451_ID, HP_FILTER_CUTOFF, 1, data, 1);
}

void mma8451_get(short *accel)
{
    unsigned char data[6];
    i2c_default_rw(MMA8451_ID, OUT_X_MSB, 6, data, 0);
    accel[1] = -(short)((data[0] << 8) | data[1]);
    accel[0] = -(short)((data[2] << 8) | data[2]);
    accel[2] = -(short)((data[4] << 8) | data[5]);
}

