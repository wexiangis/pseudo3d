/*
 *  I2C通用总线驱动
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/*
 *  返回: 0/正常
 */
int i2c_transfer_write(
    int fd,
    unsigned short slave_addr,
    unsigned char *data_buf,
    unsigned int data_len,
    unsigned char *reg_addr,
    unsigned int reg_size)
{
    int ret;
    struct i2c_rdwr_ioctl_data write_opt;
    unsigned char *writeBuff;
    unsigned int writeBuffLen;
    //
    writeBuffLen = reg_size + data_len;
    writeBuff = (unsigned char *)malloc((writeBuffLen + 1) * sizeof(unsigned char));
    memcpy(&writeBuff[0], reg_addr, reg_size);
    memcpy(&writeBuff[reg_size], data_buf, data_len);
    //
    memset(&write_opt, 0, sizeof(write_opt));
    write_opt.nmsgs = 1;
    write_opt.msgs = (struct i2c_msg *)malloc(write_opt.nmsgs * sizeof(struct i2c_msg));
    memset(write_opt.msgs, 0, write_opt.nmsgs * sizeof(struct i2c_msg));
    //
    write_opt.msgs[0].len = writeBuffLen;
    write_opt.msgs[0].flags = !I2C_M_RD;
    write_opt.msgs[0].addr = slave_addr;
    write_opt.msgs[0].buf = writeBuff;
    //
    ret = ioctl(fd, I2C_RDWR, (unsigned long)&write_opt);
    if (write_opt.nmsgs == ret)
        ret = 0;
    else
        ret = -1;
    //
    free(writeBuff);
    if (write_opt.msgs)
    {
        free(write_opt.msgs);
        write_opt.msgs = NULL;
    }
    //
    return ret;
}

/*
 *  返回: 0/正常
 */
int i2c_transfer_read(
    int fd,
    unsigned short slave_addr,
    unsigned char *data_buf,
    unsigned int data_len,
    unsigned char *reg_addr,
    unsigned int reg_size)
{
    int ret;
    struct i2c_rdwr_ioctl_data read_opt;
    //
    memset(&read_opt, 0, sizeof(read_opt));
    read_opt.nmsgs = 2;
    read_opt.msgs = (struct i2c_msg *)malloc(read_opt.nmsgs * sizeof(struct i2c_msg));
    memset(read_opt.msgs, 0, read_opt.nmsgs * sizeof(struct i2c_msg));
    //
    read_opt.msgs[0].len = reg_size;
    read_opt.msgs[0].flags = 0;
    read_opt.msgs[0].addr = slave_addr;
    read_opt.msgs[0].buf = reg_addr;
    //
    read_opt.msgs[1].len = data_len;
    read_opt.msgs[1].flags = I2C_M_RD;
    read_opt.msgs[1].addr = slave_addr;
    read_opt.msgs[1].buf = data_buf;
    //
    ret = ioctl(fd, I2C_RDWR, (unsigned long)&read_opt);
    if (read_opt.nmsgs == ret)
        ret = 0;
    else
        ret = -1;
    //
    if (read_opt.msgs)
    {
        free(read_opt.msgs);
        read_opt.msgs = NULL;
    }
    //
    return ret;
}

/*
 *  i2cPath: 例如 "/dev/i2c-0"
 * 
 *  返回: 大于0的fd
 */
int i2c_transfer_open(char *i2cPath)
{
    return open(i2cPath, O_RDWR);
}

void i2c_transfer_close(int fd)
{
    close(fd);
}

//用于STM32等设备,省去open和close操作,参数结构简化,返回0正常
static int _i2c_default_fd = 0;
int i2c_default_rw(
    unsigned char slave_addr,
    unsigned char reg_addr,
    unsigned char length,
    unsigned char *data,
    char isWrite)
{
    unsigned char reg[1] = {0};
    if(_i2c_default_fd < 1)
        _i2c_default_fd = i2c_transfer_open("/dev/i2c-1");
    if(_i2c_default_fd < 1)
        return -1;
    reg[0] = reg_addr;
    if(isWrite)
        return i2c_transfer_write(_i2c_default_fd, slave_addr, data, length, reg, 1);
    else
        return i2c_transfer_read(_i2c_default_fd, slave_addr, data, length, reg, 1);
}
