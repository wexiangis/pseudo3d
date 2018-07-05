
#ifndef _TFT_3D_H
#define _TFT_3D_H

#include "view.h"
#include <stdarg.h>     //变长参数

#define  _3D_PI     3.1415926535897

#define  _3D_Type       0   //建坐标的方式  0: x/ y| z-

#define  _3D_XYZ_ScanLen  (VIEW_X_SIZE/2)  //xyz坐标轴长度

//管理多边形图像的point-point连接关系的结构体
typedef struct _3D_PPLink{
    int point;     //选定端点
    int *target;   //要连接的目标点数组
    int targetNum; //目标数量
    struct _3D_PPLink *next;    //建立下一个连接关系
}_3D_PPLink_Type;

//comment
typedef struct _3D_Comment{
    double outXYZ[3];
    double outXYZCopy[3];
    int outXY[2];
    char *out;
    int color;
    struct _3D_Comment *next;
}_3D_Comment_Type;

//管理多边形图像的结构体
typedef struct{
    double raxyz[3];
    double mvxyz[3];
    double *array;     //x1,y1,z1;x2,y2,z2;...
    double *arrayCopy; //x1,y1,z1;x2,y2,z2;...
    int pointNum;
    int memSize;
    int *out;       //x1,y1;x2,y2;...
    int *outColor;  //col1;col2...
    //
    _3D_PPLink_Type *link;
    _3D_Comment_Type *comment;
}_3D_PointArray_Type;

_3D_PointArray_Type *_3D_pointArray_init(
    int pointNum, ...);

void _3D_reset(_3D_PointArray_Type *ddat);

void _3D_ppLink_add(
    _3D_PointArray_Type *ddat, 
    int point, int targetNum, ...);

void _3D_comment_add(
    _3D_PointArray_Type *ddat, 
    double x, double y, double z, 
    char *str, int color);

void _3D_angle_to_xyz(
    _3D_PointArray_Type *ddat);

//以下要使用tft库
void _3D_draw(
    int centreX, 
    int centreY, 
    _3D_PointArray_Type *ddat);

#endif
