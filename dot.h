/*
 *  屏幕打点工具,用来展示采样点的宏观分布情况
 */
#ifndef _DOT_H_
#define _DOT_H_

typedef struct
{
    int xOffset, yOffset;
    int width, height;
    //打点数值范围
    double xMin, xMax, yMin, yMax;
    //RGB图像矩阵
    unsigned char *map;
    int map_size;
} Dot_Struct;

Dot_Struct *dot_init(int xOffset, int yOffset, int width, int height, double xMin, double xMax, double yMin, double yMax);
void dot_release(Dot_Struct **ds);
//画点,x,y值不要超过上面的范围
void dot_set(Dot_Struct *ds, double x, double y, int color);
//清空现有的点
void dot_clear(Dot_Struct *ds);
//刷屏输出
void dot_output(Dot_Struct *ds);

#endif
