
CROSS_COMPILE=

target:
	$(CROSS_COMPILE)gcc -O3 -o out main.c view.c bmp.c gbk2312.c tft_3d.c -lm

clean:
	@rm -rf m 
