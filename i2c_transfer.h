#ifndef _I2C_TRANSFER_H_
#define _I2C_TRANSFER_H_

int i2c_transfer_open(char *i2cPath);
void i2c_transfer_close(int fd);

int i2c_transfer_write(int fd, unsigned short slave_addr, unsigned char *data_buf, unsigned int data_len, unsigned char *reg_addr, unsigned int reg_size);

int i2c_transfer_read(int fd, unsigned short slave_addr, unsigned char *data_buf, unsigned int data_len, unsigned char *reg_addr, unsigned int reg_size);

#endif
