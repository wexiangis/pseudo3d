
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "fbmap.h"
#define FB_PATH "/dev/fb0"

void fb_release(FbMap *fb)
{
    if (!fb)
        return;
    if (fb->fb)
        munmap(fb->fb, fb->fbSize);
    if (fb->fd > 0)
        close(fb->fd);
    free(fb);
    fb = NULL;
}

FbMap *fb_init(int xOffset, int yOffset)
{
    FbMap *fb = (FbMap *)calloc(1, sizeof(FbMap));

    fb->xOffset = xOffset;
    fb->yOffset = yOffset;

    fb->fd = open(FB_PATH, O_RDWR);
    if (fb->fd < 1)
    {
        fprintf(stderr, "fb_init: open %s err \r\n", FB_PATH);
        fb_release(fb);
        return NULL;
    }

    if (ioctl(fb->fd, FBIOGET_VSCREENINFO, &fb->fbInfo) < 0)
    {
        fprintf(stderr, "fb_init: ioctl FBIOGET_VSCREENINFO err \r\n");
        fb_release(fb);
        return NULL;
    }
    printf("frameBuffer: %s, %d x %d, %dbytes / %dbpp\r\n",
           FB_PATH, fb->fbInfo.xres, fb->fbInfo.yres, fb->fbInfo.bits_per_pixel / 8, fb->fbInfo.bits_per_pixel);

    fb->bpp = fb->fbInfo.bits_per_pixel / 8;
    fb->bw = fb->bpp * fb->fbInfo.xres;
    fb->bh = fb->bpp * fb->fbInfo.yres;
    fb->fbSize = fb->fbInfo.xres * fb->fbInfo.yres * fb->bpp;

    fb->fb = (unsigned char *)mmap(0, fb->fbSize, PROT_READ | PROT_WRITE, MAP_SHARED, fb->fd, 0);
    if (!fb->fb)
    {
        fprintf(stderr, "fb_init: mmap size %ld err \r\n", fb->fbSize);
        fb_release(fb);
        return NULL;
    }
    return fb;
}

void fb_refresh(FbMap *fb, unsigned char *data, int width, int height, int per)
{
    int x, y, offset, bmpCount;

    if (!fb)
        return;

    bmpCount = 0;
    for (y = 0; y < height; y++)
    {
        offset = (y + fb->yOffset) * fb->bw + (0 + fb->xOffset) * fb->bpp;
        for (x = 0; x < width; x++)
        {
            fb->fb[offset + 3] = 0x00;             //A
            fb->fb[offset + 2] = data[bmpCount++]; //R
            fb->fb[offset + 1] = data[bmpCount++]; //G
            fb->fb[offset + 0] = data[bmpCount++]; //B
            offset += fb->bpp;
        }
    }
}
