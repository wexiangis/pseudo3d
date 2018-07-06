
CROSS_COMPILE=

target:
	$(CROSS_COMPILE)gcc -O3 -o out main.c view.c bmp.c gbk2312.c pseudo3D.c -lm

clean:
	@rm -rf m 
