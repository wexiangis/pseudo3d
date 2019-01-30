
#ifndef _PSEUDO3D_H
#define _PSEUDO3D_H

#include "view.h"
#include <stdarg.h>     //变长参数

#define  _3D_PI     3.14159265358979323846

#define  _3D_Type   0   //建坐标的方式  0: x/ y| z-

#define  _3D_XYZ_ScanLen  (VIEW_X_SIZE/2)  //xyz坐标轴长度

//管理多边形图像的point-point连接关系的结构体
typedef struct _3D_PPLink{
    int order;               //选定端点
    int *targetOrderArray;   //要连接的目标点数组
    int targetOrderNum;      //目标数量
    int color;
    struct _3D_PPLink *next; //建立下一个连接关系
}_3D_PPLink_Type;

//comment
typedef struct _3D_Comment{
    double xyz[3];
    double xyzCopy[3];
    int xy[2];
    char *comment;
    int type;    //0/普通字符串(添加后固定不变)  1/传入指针
    int color;
    struct _3D_Comment *next;
}_3D_Comment_Type;

//管理多边形图像的结构体
typedef struct{
    double raxyz[3];
    double mvxyz[3];
    double *xyzArray;     //x1,y1,z1;x2,y2,z2;...
    double *xyzArrayCopy; //x1,y1,z1;x2,y2,z2;...
    int pointNum;
    int xyzArrayMemSize;
    int *xyArray;         //x1,y1;x2,y2;...
    int *color;           //col1;col2...
    //
    _3D_PPLink_Type *link;
    _3D_Comment_Type *comment;
}_3D_PointArray_Type;

_3D_PointArray_Type *_3D_pointArray_init(
    int pointNum, 
    double x, 
    double y, 
    double z, 
    int color, ...);

void _3D_reset(_3D_PointArray_Type *dpat);

void _3D_ppLink_add(
    _3D_PointArray_Type *dpat, 
    int color, 
    int order, 
    int targetOrderNum, ...);

void _3D_comment_add(
    _3D_PointArray_Type *dpat, 
    double x, 
    double y, 
    double z, 
    char *comment, 
    int type, 
    int color);

void _3D_angle_to_xyz(
    _3D_PointArray_Type *dpat);

//以下要使用tft库
void _3D_draw(
    int centreX, 
    int centreY, 
    _3D_PointArray_Type *dpat);

#endif
