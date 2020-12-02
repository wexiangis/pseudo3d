/*
 *  I2C通用总线驱动
 */
#ifndef _I2C_TRANSFER_H_
#define _I2C_TRANSFER_H_

int i2c_transfer_open(char *i2cPath);
void i2c_transfer_close(int fd);

int i2c_transfer_write(
    int fd,
    unsigned short slave_addr,
    unsigned char *data_buf,
    unsigned int data_len,
    unsigned char *reg_addr,
    unsigned int reg_size);
int i2c_transfer_read(
    int fd,
    unsigned short slave_addr,
    unsigned char *data_buf,
    unsigned int data_len,
    unsigned char *reg_addr,
    unsigned int reg_size);

//用于STM32等设备,省去open和close操作,参数结构简化,返回0正常
int i2c_default_rw(
    unsigned char slave_addr,
    unsigned char reg_addr,
    unsigned char length,
    unsigned char *data,
    char isWrite);

#endif
