/*
 *  fb矩阵输出
 */
#ifndef _FBMAP_H_
#define _FBMAP_H_

#include <stdlib.h>
#include <linux/fb.h>

typedef struct
{
    int fd;
    unsigned char *fb;
    size_t fbSize;
    struct fb_var_screeninfo fbInfo;
    //bytes per point
    int bpp;
    //bytes width, height
    int bw, bh;
    //draw offset
    int xOffset, yOffset;
} FbMap;

FbMap *fb_init(int xOffset, int yOffset);
void fb_release(FbMap *fb);

void fb_refresh(FbMap *fb, unsigned char *data, int width, int height, int per);

#endif