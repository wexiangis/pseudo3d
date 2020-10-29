/*
 *  屏幕打点工具,用来展示采样点的宏观分布情况
 */
#include <stdlib.h>
#include <string.h>
#include "dot.h"
#include "fbmap.h"

Dot_Struct *dot_init(int xOffset, int yOffset, int width, int height, double xMin, double xMax, double yMin, double yMax)
{
    Dot_Struct *ds = (Dot_Struct *)calloc(1, sizeof(Dot_Struct));
    ds->xOffset = xOffset;
    ds->yOffset = yOffset;
    ds->width = width;
    ds->height = height;
    ds->xMin = xMin;
    ds->xMax = xMax;
    ds->yMin = yMin;
    ds->yMax = yMax;
    ds->map_size = width * height * 3;
    ds->map = (unsigned char *)calloc(ds->map_size, sizeof(char));
    return ds;
}

void dot_release(Dot_Struct **ds)
{
    if (!ds)
        return;
    if (*ds)
    {
        if ((*ds)->map)
            free((*ds)->map);
        free(*ds);
        *ds = NULL;
    }
}

void dot_set(Dot_Struct *ds, double x, double y, int color)
{
    int offset = 0;
    int _x, _y;
    if (!ds)
        return;
    _x = (int)((x - ds->xMin) * ds->width / (ds->xMax - ds->xMax));
    _y = (int)((y - ds->yMin) * ds->height / (ds->yMax - ds->yMin));
    if (_x < 0)
        _x = 0;
    else if (_x >= ds->width)
        _x = ds->width - 1;
    if (_y < 0)
        _y = 0;
    else if (_y >= ds->height)
        _y = ds->height - 1;
    offset = (_y * ds->width + _x) * 3;
    ds->map[offset] = (((color >> 16) & 0xFF) >> 1) + (ds->map[offset] >> 1);
    offset += 1;
    ds->map[offset] = (((color >> 8) & 0xFF) >> 1) + (ds->map[offset] >> 1);
    offset += 1;
    ds->map[offset] = (((color >> 0) & 0xFF) >> 1) + (ds->map[offset] >> 1);
}

void dot_clear(Dot_Struct *ds)
{
    if (!ds)
        return;
    memset(ds->map, 0, ds->map_size);
}

void dot_output(Dot_Struct *ds)
{
    if (!ds)
        return;
    fb_output(ds->map, ds->xOffset, ds->yOffset, ds->width, ds->height);
}
