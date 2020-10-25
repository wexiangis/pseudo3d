/*
 *  xxx
 */
#ifndef _DOT_H_
#define _DOT_H_

#define DOT_X_RANGE_START (-1.0)
#define DOT_X_RANGE_END   (1.0)

#define DOT_Y_RANGE_START (-1.0)
#define DOT_Y_RANGE_END   (1.0)

void dot_set(double x, double y, int color);

void dot_clear(void);

void dot_refresh(void);

#endif 
