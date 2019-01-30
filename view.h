
#ifndef _VIEW_H
#define _VIEW_H

#include "bmp.h"
#include "gbk2312.h"

#define  VIEW_X_SIZE    320
#define  VIEW_Y_SIZE    320
#define  VIEW_PICTURE_PERW     3

void amoled_print_dot(int x, int y, const unsigned char *data);
void amoled_print_dot2(int x, int y, const unsigned char *data);
void amoled_print_en(void);
void amoled_print_clear(void);

#define  PRINT_DOT(x,y,data)   amoled_print_dot(x-1,y-1,data)       //定点写
#define  PRINT_DOT2(x,y,data)  amoled_print_dot2(x-1,y-1,data)
#define  PRINT_EN()            amoled_print_en()                      //使能输出
#define  PRINT_CLEAR()         amoled_print_clear()  //清屏
#define  VIEW_DIR(x)                  ;//amoled_displayDir(x)                //屏幕方向重设   0/正  1/右  2/倒  3/左  其它/正
#define  VIEW_MODE(x)             ;//amoled_modeSet(x)        //0/初始化(亮屏), 1/亮屏, 2/灭屏, 3/wakeup, 4/sleep, 5/powerOff, 6/刷新屏幕
#define  VIEW_BRIGHT(x)           ;//amoled_brightSet(x)       // 0 ~ 0xFF

//view common
//view common
void view_dot(long color, int xStart, int yStart, int size);
void view_line(long color, int xStart, int yStart, int xEnd, int yEnd, int size, int space);
void view_circle(long color, int xStart, int yStart, int rad, int size);
void view_circleLoop(long color, int xStart, int yStart, int rad, int size, int div, int divStart, int divEnd);
void view_rectangle(
    long color, 
    int xStart, int yStart, 
    int xEnd, int yEnd, 
    int size, int rad, int mode,
    int minX, int minY, 
    int maxX, int maxY);     //mode 0:按照size画框  1:实心填充  2:半透填充
void view_rectangle_padding(const unsigned char *pic, int xStart, int yStart, int xEnd, int yEnd);
void view_parallelogram(
    long color, 
    int xStart, int yStart, 
    int xEnd, int yEnd, 
    int size, int width, int mode, 
    int minX, int minY, 
    int maxX, int maxY);     //mode 0:按照size画框  1:实心填充  2:半透填充
void view_string_print(unsigned char *fColor, unsigned char *bColor, unsigned char *buf, unsigned int bufLen, int width, int xStart, int yStart);
void view_string(long fColor, long bColor, char *str, int xStart, int yStart, int type, int space);
void view_string_rectangle(
    long fColor, long bColor, 
    char *str, 
    int xStart, int yStart, 
    int strWidth, int strHight, 
    int xScreenStart, int yScreenStart, 
    int xScreenEnd, int yScreenEnd, 
    int type, int space, int printMode);
int view_string_rectangleCR(
    long fColor, long bColor, 
    char *str, 
    int xStart, int yStart, 
    int strWidth, int strHight, 
    int xScreenStart, int yScreenStart, 
    int xScreenEnd, int yScreenEnd, 
    int type, int space, int xErr, int printMode);
int view_string_rectangleMultiLine(
    long fColor, long bColor, 
    char *str, 
    int *xStart, int *yStart, 
    int *strWidth, int *strHight, 
    int type, int space, int lineNum, int *retLineCharNum, int printMode);

#endif

