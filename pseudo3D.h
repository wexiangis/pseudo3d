
/*
 *  自制乞丐版3D引擎
 */
#ifndef _PSEUDO3D_H
#define _PSEUDO3D_H

#include "view.h"
#include <stdarg.h> //变长参数

#define _3D_PI 3.14159265358979323846 //位数越多 精度越高

#define _3D_Type 0 //建坐标的方式  0: x/ y| z-

#define _3D_XYZ_ScanLen (VIEW_X_SIZE / 2) //xyz坐标轴长度

//管理多边形图像的point-point连接关系的结构体
typedef struct _3D_PPLink
{
    int order;             //选定端点
    int *targetOrderArray; //要连接的目标点数组
    int targetOrderNum;    //目标数量
    int color;
    struct _3D_PPLink *next; //建立下一个连接关系
} _3D_PPLink_Type;

//注释信息结构体
typedef struct _3D_Comment
{
    double xyz[3];     //所在三维坐标
    double xyzCopy[3]; //备份数组
    int xy[2];         //二维投影点坐标
    char *comment;     //注释内容
    int type;          //0/普通字符串(添加后固定不变)  1/传入指针
    int color;
    struct _3D_Comment *next; //链表
} _3D_Comment_Type;

//管理多边形图像的结构体
typedef struct
{
    double raxyz[3];      //xyz轴旋转量(0~2pi)
    double mvxyz[3];      //xyz轴的平移量
    double *xyzArray;     //3元数组 x1,y1,z1;x2,y2,z2;...
    double *xyzArrayCopy; //3元数组 x1,y1,z1;x2,y2,z2;... 备份数组
    int pointNum;         //上面两数组的点数量
    int xyzArrayMemSize;  //上面两数组的实际内存长度(字节数)
    int *xyArray;         //2元数组 x1,y1;x2,y2;... 三维坐标投影到二维后的坐标
    int *color;           //1元数组 col1;col2... 二维投影点的颜色

    _3D_PPLink_Type *link;     //各三维点的连线关系
    _3D_Comment_Type *comment; //注释信息
} _3D_PointArray_Type;

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
