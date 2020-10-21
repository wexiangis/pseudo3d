
CROSS_COMPILE=

obj-c += main.c
obj-c += delayus.c

# 图像输出到fb0
obj-c += view.c
# bmp文件读写
obj-c += bmp.c
# 根据utf8字符串获取文字点阵
obj-c += gbk2312.c
# 自制乞丐版3D引擎
obj-c += pseudo3d.c
# I2C通用总线驱动
obj-c += i2c_transfer.c
# fb矩阵输出
obj-c += fbmap.c
# 自制简易版示波器,图像输出到fb0
obj-c += wave.c
# 根据MPU6050数据计算姿态
obj-c += posture.c

# MPU6050 驱动
obj-c += mpu6050/mpu6050.c
obj-c += mpu6050/inv_mpu.c
obj-c += mpu6050/inv_mpu_dmp_motion_driver.c

# HMC5883 驱动
obj-c += hmc5883.c

target:
	$(CROSS_COMPILE)gcc -Wall -o out $(obj-c) -I./ -I./mpu6050 -lm -lpthread

clean:
	@rm -rf out
