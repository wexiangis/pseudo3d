
/*
 *  自制乞丐版3D引擎
 */
#ifndef _PSEUDO3D_H
#define _PSEUDO3D_H

#include <stdarg.h> //变长参数
#include "view.h"

#define P3D_PI 3.14159265358979323846
#define P3D_2PI (P3D_PI * 2)

//管理多边形图像的point-point连接关系的结构体
typedef struct P3D_PPLink
{
    int order;             //选定端点
    int *targetOrderArray; //要连接的目标点数组
    int targetOrderNum;    //目标数量
    int color;
    struct P3D_PPLink *next; //建立下一个连接关系
} P3D_PPLink_Type;

//注释信息结构体
typedef struct P3D_Comment
{
    float xyz[3];     //所在三维坐标
    float xyzCopy[3]; //备份数组
    int xy[2];         //二维投影点坐标
    char *comment;     //注释内容
    int type;          //0/普通字符串(添加后固定不变)  1/传入指针
    int color;
    struct P3D_Comment *next; //链表
} P3D_Comment_Type;

//管理多边形图像的结构体
typedef struct
{
    float raxyz[3];      //xyz轴旋转量(0~2pi)
    float mvxyz[3];      //xyz轴的平移量
    float *xyzArray;     //3元数组 x1,y1,z1;x2,y2,z2;...
    float *xyzArrayCopy; //3元数组 x1,y1,z1;x2,y2,z2;... 备份数组
    int pointNum;         //上面两数组的点数量
    int xyzArrayMemSize;  //上面两数组的实际内存长度(字节数)
    int *xyArray;         //2元数组 x1,y1;x2,y2;... 三维坐标投影到二维后的坐标
    int *color;           //1元数组 col1;col2... 二维投影点的颜色
    P3D_PPLink_Type *link;     //各三维点的连线关系
    P3D_Comment_Type *comment; //注释信息
    int _matrix_mode;     //0/使用xyz旋转矩阵 1/使用zyx旋转矩阵
} P3D_PointArray_Type;

/*
 *  图形建模
 *  参数:
 *      pointNum: 三维坐标点数量
 *      x, y, z， color: 循环填入三维坐标点和点的颜色
 * 
 *  注意: x, y, z 必须用 0.00 的格式赋值, 例如: 3 写成 3.00, -13 写成 -13.00
 */
P3D_PointArray_Type *p3d_init(
    int pointNum,
    float x,
    float y,
    float z,
    int color, ...);

/*
 *  重置三维图形到初始化时的状态
 */
void p3d_reset(P3D_PointArray_Type *dpat);

/*
 *  添加三维坐标点连线关系
 *  参数:
 *      color: 线颜色
 *      point: 顶点所在序号,从0数起
 *      targetNum: 和顶点相连的点的数量
 *      ...: 输入要和顶点连接的点所在序号,从0数起
 */
void p3d_ppLink_add(
    P3D_PointArray_Type *dpat,
    int color,
    int order,
    int targetOrderNum, ...);

/*
 *  对图形添加注释信息
 *  参数:
 *      x, y, z: 注释在三维空间中的位置
 *      comment: 注释内容
 *      type: 注释类型,0/普通字符串(添加后固定不变)  1/传入指针
 *      color: 文字颜色
 */
void p3d_comment_add(
    P3D_PointArray_Type *dpat,
    float x,
    float y,
    float z,
    char *comment,
    int type,
    int color);

/*
 *  输出当前图形到屏幕
 *  参数:
 *      centreX, centreY: 绘制原点在屏幕中的位置,一般为(屏幕宽/2, 屏幕高/2)
 */
void p3d_draw(
    int centreX,
    int centreY,
    P3D_PointArray_Type *dpat);

/*
 *  把空间点(z,y,z)和原点连线画在屏幕上
 *  参数:
 *      centreX, centreY: 绘制原点在屏幕中的位置,一般为(屏幕宽/2, 屏幕高/2)
 *      color: 划线颜色
 */
void p3d_draw2(
    int centreX,
    int centreY,
    int color, 
    float *xyz);

#endif
