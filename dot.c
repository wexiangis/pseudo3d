/*
 *  屏幕打点工具,用来展示采样点的宏观分布情况
 */
#include <stdlib.h>
#include <string.h>
#include "dot.h"
#include "fbmap.h"

//画布在屏幕的起始位置
#define DOT_X_OFFSET 0
#define DOT_Y_OFFSET 0

//画布长高
static int dot_x_size = 320;
static int dot_y_size = 320;

static int dot_x_half = 0;
static int dot_y_half = 0;

static FbMap *dot_fbmap = NULL;
static unsigned char *dot_data;
static int dot_data_size = 0;

void dot_init(void)
{
    if(dot_fbmap)
        return;
    dot_fbmap = fb_init(DOT_X_OFFSET, DOT_Y_OFFSET);
    if (!dot_fbmap)
        return;
    if (dot_x_size > dot_fbmap->fbInfo.xres - DOT_X_OFFSET)
        dot_x_size = dot_fbmap->fbInfo.xres - DOT_X_OFFSET;
    if (dot_y_size > dot_fbmap->fbInfo.yres - DOT_Y_OFFSET)
        dot_y_size = dot_fbmap->fbInfo.yres - DOT_Y_OFFSET;
    dot_x_half = dot_x_size / 2;
    dot_y_half = dot_y_size / 2;
    dot_data_size = dot_x_size * dot_y_size * 3;
    dot_data = (unsigned char*)calloc(dot_data_size, sizeof(unsigned char));
}

void dot_set(double x, double y, int color)
{
    int offset = 0;
    int _x = (int)((x - DOT_X_RANGE_START) * dot_x_size / (DOT_X_RANGE_END - DOT_X_RANGE_START));
    int _y = (int)((y - DOT_Y_RANGE_START) * dot_y_size / (DOT_Y_RANGE_END - DOT_Y_RANGE_START));
    if (_x < 0) _x = 0;
    else if(_x >= dot_x_size) _x = dot_x_size - 1;
    if (_y < 0) _y = 0;
    else if(_y >= dot_y_size) _y = dot_y_size - 1;
    offset = (_y * dot_x_size + _x) * 3;
    dot_data[offset] = (((color >> 16) & 0xFF) >> 1) + (dot_data[offset] >> 1);
    offset += 1;
    dot_data[offset] = (((color >> 8) & 0xFF) >> 1) + (dot_data[offset] >> 1);
    offset += 1;
    dot_data[offset] = (((color >> 0) & 0xFF) >> 1) + (dot_data[offset] >> 1);
}

void dot_clear(void)
{
    dot_init();
    memset(dot_data, 0, dot_data_size);
}

void dot_refresh(void)
{
    dot_init();
    fb_refresh(dot_fbmap, dot_data, dot_x_size, dot_y_size, 3);
}
