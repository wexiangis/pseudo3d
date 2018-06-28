
#ifndef _BMP_H
#define _BMP_H

unsigned char *bmp_get(char *filePath, int *picMaxSize, int *width, int *height, int *per);
int bmp_create(char *filePath, unsigned char *data, int width, int height, int per);

#endif

