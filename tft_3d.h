
#ifndef _TFT_3D_H
#define _TFT_3D_H

#include "view.h"
#include <stdarg.h>     //变长参数

#define  _3D_PI     3.1415926535897

#define  _3D_Type       0   //建坐标的方式  0: x/ y| z-

#define  _3D_XYZ_ScanLen    100          //xyz坐标轴长度
#define  _3D_XYZ_LineLen    80           //动态3根线的长度

#define  _3D_Angle      (_3D_PI/4)       //空间直角坐标系的夹角      //这里为45度
#define  _3D_Pd         0.71             //45度视角下斜线长度比例    //

//管理多边形图像的point-point连接关系的结构体
typedef struct _3D_PPLink{
    int point;     //选定端点
    int *target;   //要连接的目标点数组
    int targetNum; //目标数量
    struct _3D_PPLink *next;    //建立下一个连接关系
}_3D_PPLink_Type;

//管理多边形图像的结构体
typedef struct{
    double raxyz[3];
    double *array;     //x1,y1,z1;x2,y2,z2;...
    double *arrayCopy; //x1,y1,z1;x2,y2,z2;...
    int pointNum;
    int memSize;
    int *out;       //x1,y1;x2,y2;...
    int *outColor;  //col1;col2...
    //
    _3D_PPLink_Type *link;
}_3D_DotArray_Type;

_3D_DotArray_Type *_3D_pointArray_init(
    int pointNum, ...);

void _3D_ppLink_add(
    _3D_DotArray_Type *ddat, 
    int point, int targetNum, ...);

void _3D_xyz_to_xy(
    double _3D_XYZ[3], int _2D_XY[2]);

void _3D_angle_to_xyz(
    _3D_DotArray_Type *ddat);

//以下要使用tft库
void _3D_draw(
    int centreX, 
    int centreY, 
    _3D_DotArray_Type *ddat);

#endif
