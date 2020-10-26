/*
 *  屏幕打点工具,用来展示采样点的宏观分布情况
 */
#ifndef _DOT_H_
#define _DOT_H_

//横向数值范围
#define DOT_X_RANGE_START (-1.0)
#define DOT_X_RANGE_END   (1.0)

//纵向数值范围
#define DOT_Y_RANGE_START (-1.0)
#define DOT_Y_RANGE_END   (1.0)

//画点,x,y值不要超过上面的范围
void dot_set(double x, double y, int color);

//清空现有的点
void dot_clear(void);

//刷屏输出
void dot_refresh(void);

#endif 
