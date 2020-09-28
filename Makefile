
CROSS_COMPILE=

obj-c += main.c
obj-c += view.c
obj-c += bmp.c
obj-c += gbk2312.c
obj-c += pseudo3D.c

obj-c += i2c_transfer.c
obj-c += mpu6050.c
obj-c += fbmap.c
obj-c += wave.c

target:
	$(CROSS_COMPILE)gcc -Wall -o out $(obj-c) -lm

clean:
	@rm -rf out
