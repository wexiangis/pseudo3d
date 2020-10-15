
/*
 *  自制乞丐版3D引擎
 */
#include "pseudo3D.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 0:增量式,适合连续变化的旋转(会累积误差)
// 1:每次都从原始坐标进行XYZ依次旋转,适合一次旋转
#define _3D_MODE_SWITCH 1

//三维坐标轴的原点和三维图形的各个顶点连线
#define _3D_DRAW_PO_LINE 0 //point-origin connect

//自动把输入的各个点依次连线
#define _3D_DRAW_PP_LINK 0 //point-point connect
#if (_3D_DRAW_PP_LINK)
//自动把最后一个点和第一个点连线
#define _3D_DRAW_PP_LINK_LAST 0 //last point-first point connect
#endif

//画线的线宽
#define _3D_LINE_SIZE 1

//三维投影二维时各轴向长度的缩放比例
#define _3D_PD_X 0.6
#define _3D_PD_Y 1.0
#define _3D_PD_Z 1.0

/*
 *  重置三维图形到初始化时的状态
 */
void _3D_reset(_3D_PointArray_Type *dpat)
{
    _3D_Comment_Type *dct;
    //
    memcpy(dpat->xyzArray, dpat->xyzArrayCopy, dpat->xyzArrayMemSize);
    memset(dpat->xyArray, 0, dpat->pointNum * 2 * sizeof(int));
    memset(dpat->raxyz, 0, sizeof(dpat->raxyz));
    memset(dpat->mvxyz, 0, sizeof(dpat->mvxyz));
    //
    dct = dpat->comment;
    while (dct)
    {
        memcpy(dct->xyz, dct->xyzCopy, sizeof(dct->xyz));
        memset(dct->xy, 0, sizeof(dct->xy));
        //
        dct = dct->next;
    }
}

/*
 *  添加三维坐标点连线关系
 *  参数:
 *      color: 线颜色
 *      point: 顶点所在序号,从0数起
 *      targetNum: 和顶点相连的点的数量
 *      ...: 输入要和顶点连接的点所在序号,从0数起
 */
void _3D_ppLink_add(_3D_PointArray_Type *dpat, int color, int point, int targetNum, ...)
{
    int i, count, tempTarget;
    _3D_PPLink_Type *dpplt;
    va_list ap;
    //
    if (dpat == NULL || point >= dpat->pointNum)
        return;
    //还没开辟内存
    if (dpat->link == NULL)
    {
        dpplt = dpat->link = (_3D_PPLink_Type *)calloc(1, sizeof(_3D_PPLink_Type));
        dpplt = dpat->link;
    }
    else
    {
        dpplt = dpat->link;
        //移至末尾
        while (dpplt->next)
            dpplt = dpplt->next;
        //末尾新建
        dpplt->next = (_3D_PPLink_Type *)calloc(1, sizeof(_3D_PPLink_Type));
        dpplt = dpplt->next;
    }
    //
    va_start(ap, targetNum);
    dpplt->order = point;
    dpplt->targetOrderArray = (int *)calloc(targetNum + 1, sizeof(int));
    dpplt->color = color;
    for (i = 0, count = 0; i < targetNum; i++)
    {
        tempTarget = va_arg(ap, int);
        if (tempTarget < dpat->pointNum)
        {
            dpplt->targetOrderArray[count] = tempTarget;
            count += 1;
        }
    }
    dpplt->targetOrderNum = count;
}

/*
 *  对图形添加注释信息
 *  参数:
 *      x, y, z: 注释在三维空间中的位置
 *      comment: 注释内容
 *      type: 注释类型,0/普通字符串(添加后固定不变)  1/传入指针
 *      color: 文字颜色
 */
void _3D_comment_add(_3D_PointArray_Type *dpat, double x, double y, double z, char *comment, int type, int color)
{
    _3D_Comment_Type *dct;
    //
    if (dpat == NULL || comment == NULL)
        return;
    //
    if (dpat->comment == NULL)
    {
        dpat->comment = (_3D_Comment_Type *)calloc(1, sizeof(_3D_Comment_Type));
        dct = dpat->comment;
    }
    else
    {
        dct = dpat->comment;
        while (dct->next)
            dct = dct->next;
        dct->next = (_3D_Comment_Type *)calloc(1, sizeof(_3D_Comment_Type));
        dct = dct->next;
    }
    //
    dct->xyz[0] = dct->xyzCopy[0] = x;
    dct->xyz[1] = dct->xyzCopy[1] = y;
    dct->xyz[2] = dct->xyzCopy[2] = z;
    dct->type = type;
    if (dct->type == 0)
    {
        dct->comment = (char *)calloc(strlen(comment) + 16, sizeof(char));
        strcpy(dct->comment, comment);
    }
    else
        dct->comment = comment;
    dct->color = color;
}

/*
 *  图形建模
 *  参数:
 *      pointNum: 三维坐标点数量
 *      x, y, z， color: 循环填入三维坐标点和点的颜色
 * 
 *  注意: x, y, z 必须用 0.00 的格式赋值, 例如: 3 写成 3.00, -13 写成 -13.00
 */
_3D_PointArray_Type *_3D_pointArray_init(int pointNum, double x, double y, double z, int color, ...)
{
    int i, j;
    _3D_PointArray_Type *dpat;
    va_list ap;
    //
    if (pointNum < 1)
        return NULL;
    //
    dpat = (_3D_PointArray_Type *)calloc(1, sizeof(_3D_PointArray_Type));
    //
    dpat->pointNum = pointNum;
    dpat->xyzArrayMemSize = pointNum * 3 * sizeof(double);
    //
    dpat->xyzArray = (double *)calloc(pointNum * 3 + 3, sizeof(double));
    dpat->xyzArrayCopy = (double *)calloc(pointNum * 3 + 3, sizeof(double));
    //
    dpat->xyArray = (int *)calloc(pointNum * 2 + 2, sizeof(int));
    dpat->color = (int *)calloc(pointNum + 1, sizeof(int));
    //
    dpat->xyzArray[0] = x;
    dpat->xyzArray[1] = y;
    dpat->xyzArray[2] = z;
    dpat->color[0] = color;
    va_start(ap, color);
    for (i = 1, j = 3; i < pointNum; i++)
    {
        dpat->xyzArray[j] = va_arg(ap, double);
        dpat->xyzArray[j + 1] = va_arg(ap, double);
        dpat->xyzArray[j + 2] = va_arg(ap, double);
        dpat->color[i] = va_arg(ap, int);
        j += 3;
    }
    va_end(ap);
    memcpy(dpat->xyzArrayCopy, dpat->xyzArray, dpat->xyzArrayMemSize);
    //
    return dpat;
}

/*
 *  把三维坐标点投影到二维坐标点上
 */
void _3D_xyz_to_xy(double _3D_XYZ[3], int _2D_XY[2])
{
    double tempX, tempY;
    double x, y, z;
    //
    x = _3D_XYZ[0];
    y = _3D_XYZ[1];
    z = _3D_XYZ[2];
    //
    if (_3D_Type == 0)
    {
        // Y
        _2D_XY[0] = y * _3D_PD_Y;

        // Z
        _2D_XY[1] = z * _3D_PD_Z;

        // X
        if (x != 0)
        {
            tempX = x * _3D_PD_X;
            tempY = x * (1 - _3D_PD_X);
            //
            _2D_XY[0] -= (int)tempX;
            _2D_XY[1] -= (int)tempY;
        }
    }
}

/*
 *  point[3]绕自身坐标轴旋转raxyz[3]量,输出坐标复写到point[3]中
 *  参数:
 *      raxyz[3] : 绕X/Y/Z轴的转角(rad: 0~2pi)
 *      point[3] : 要修正的空间向量的坐标
 * 
 *  备注: 绕自身旋转使用左乘, 绕大地坐标旋转使用右乘, 这里为左乘
 */
void _3D_angle_to_xyz0(double raxyz[3], double point[3])
{
    double x, y, z;
    double Xrad, Yrad, Zrad;

    x = point[0];
    y = point[1];
    z = point[2];
    Xrad = raxyz[0];
    Yrad = raxyz[1];
    Zrad = raxyz[2];

    /*      [roll X]
    *   1       0       0
    *   0     cosA    -sinA
    *   0     sinA     cosA
    *
    *       [roll Y]
    *  cosB     0      sinB
    *   0       1       0
    * -sinB     0      cosB
    *
    *       [roll Z]
    *  cosC   -sinC     0
    *  sinC    cosC     0
    *   0       0       1
    *
    *                                    |x|
    *   result = [roll X][roll Y][roll Z]|y|
    *                                    |z|
    *            |point[0]|
    *          = |point[1]|
    *            |point[2]|
    *
    *   point[*] just like the following ...
    */

    point[0] =
        x * cos(Yrad) * cos(Zrad) -
        y * cos(Yrad) * sin(Zrad) +
        z * sin(Yrad);
    point[1] =
        x * (sin(Xrad) * sin(Yrad) * cos(Zrad) + cos(Xrad) * sin(Zrad)) -
        y * (sin(Xrad) * sin(Yrad) * sin(Zrad) - cos(Xrad) * cos(Zrad)) -
        z * sin(Xrad) * cos(Yrad);
    point[2] =
        -x * (cos(Xrad) * sin(Yrad) * cos(Zrad) - sin(Xrad) * sin(Zrad)) +
        y * (cos(Xrad) * sin(Yrad) * sin(Zrad) + sin(Xrad) * cos(Zrad)) +
        z * cos(Xrad) * cos(Yrad);
}

/*
 *  根据当前 dpat 中的 raxyz[3] 和 mvxyz[3] 对当前的图形进行旋转和平移
 */
void _3D_angle_to_xyz(_3D_PointArray_Type *dpat)
{
    int i, j;
    _3D_Comment_Type *dct;
    //
    if (dpat == NULL ||
        dpat->xyzArray == NULL ||
        dpat->xyzArrayCopy == NULL)
        return;
    //
    if (dpat->raxyz[0] >= 2 * _3D_PI)
        dpat->raxyz[0] -= 2 * _3D_PI;
    else if (dpat->raxyz[0] < 0)
        dpat->raxyz[0] += 2 * _3D_PI;

    if (dpat->raxyz[1] >= 2 * _3D_PI)
        dpat->raxyz[1] -= 2 * _3D_PI;
    else if (dpat->raxyz[1] < 0)
        dpat->raxyz[1] += 2 * _3D_PI;

    if (dpat->raxyz[2] >= 2 * _3D_PI)
        dpat->raxyz[2] -= 2 * _3D_PI;
    else if (dpat->raxyz[2] < 0)
        dpat->raxyz[2] += 2 * _3D_PI;

    //
    for (i = 0, j = 0; i < dpat->pointNum; i++)
    {
#if (_3D_MODE_SWITCH)
        //mode/0: 使用原始的坐标和累积的转角量,一次转换到目标坐标
        memcpy(&dpat->xyzArray[j], &dpat->xyzArrayCopy[j], 3 * sizeof(double));
#endif
        //
        _3D_angle_to_xyz0(dpat->raxyz, &dpat->xyzArray[j]);
        //
        dpat->xyzArray[j] += dpat->mvxyz[0];
        dpat->xyzArray[j + 1] += dpat->mvxyz[1];
        dpat->xyzArray[j + 2] += dpat->mvxyz[2];
        //
        j += 3;
    }
    //
    dct = dpat->comment;
    while (dct)
    {
#if (_3D_MODE_SWITCH)
        //mode/0: 使用原始的坐标和累积的转角量,一次转换到目标坐标
        memcpy(dct->xyz, dct->xyzCopy, 3 * sizeof(double));
#endif
        //
        _3D_angle_to_xyz0(dpat->raxyz, dct->xyz);
        //
        dct->xyz[0] += dpat->mvxyz[0];
        dct->xyz[1] += dpat->mvxyz[1];
        dct->xyz[2] += dpat->mvxyz[2];
        //
        dct = dct->next;
    }

#if (!_3D_MODE_SWITCH)
    //mode/1: 每次转换都使用的上次转换的坐标,转角量使用过后清零
    memset(dpat->raxyz, 0, sizeof(dpat->raxyz));
    memset(dpat->mvxyz, 0, sizeof(dpat->mvxyz));
#endif
}


/*
 *  输出当前图形到屏幕
 *  参数:
 *      centreX, centreY: 绘制原点在屏幕中的位置,一般为(屏幕宽/2, 屏幕高/2)
 */
void _3D_draw(int centreX, int centreY, _3D_PointArray_Type *dpat)
{
    int i, j, k;
    _3D_PPLink_Type *dpplt;
    int mP, mT;
    _3D_Comment_Type *dct;

    if (dpat == NULL)
        return;

    //三维坐标转二维
    for (i = 0, j = 0, k = 0; i < dpat->pointNum; i++)
    {
        _3D_xyz_to_xy(&dpat->xyzArray[j], &dpat->xyArray[k]);

        dpat->xyArray[k] = centreX - dpat->xyArray[k];
        dpat->xyArray[k + 1] = centreY - dpat->xyArray[k + 1];
        // printf("2DXY: %d / %d\r\n", dpat->xyArray[k], dpat->xyArray[k+1]);
        view_dot(dpat->color[i], dpat->xyArray[k], dpat->xyArray[k + 1], 2);
        //
        j += 3;
        k += 2;
    }

    //原点和各个点连线
#if (_3D_DRAW_PO_LINE)
    for (i = 0, j = 0; i < dpat->pointNum; i++)
    {
        view_line(dpat->color[i],
                  centreX, centreY,
                  dpat->xyArray[j], dpat->xyArray[j + 1],
                  _3D_LINE_SIZE, 0);
        j += 2;
    }
#endif

    //点和点的连线
    if (dpat->pointNum > 1)
    {
#if (_3D_DRAW_PP_LINK)
        //前一点和下一点连线
        for (i = 0, j = 0; i < dpat->pointNum - 1; i++)
        {
            view_line(
                (dpat->color[i] + dpat->color[i + 1]) / 2,
                dpat->xyArray[j], dpat->xyArray[j + 1],
                dpat->xyArray[j + 2], dpat->xyArray[j + 3],
                _3D_LINE_SIZE, 0);
            j += 2;
        }
#if (_3D_DRAW_PP_LINK_LAST)
        //最后一点和第一点连线
        view_line(
            (dpat->color[0] + dpat->color[dpat->pointNum - 1]) / 2,
            dpat->xyArray[0], dpat->xyArray[1],
            dpat->xyArray[(dpat->pointNum - 1) * 2], dpat->xyArray[(dpat->pointNum - 1) * 2 + 1],
            _3D_LINE_SIZE, 0);
#endif
#endif
        //根据ppLink关系连线
        if ((dpplt = dpat->link))
        {
            while (dpplt)
            {
                mP = dpplt->order * 2;
                for (i = 0; i < dpplt->targetOrderNum; i++)
                {
                    mT = dpplt->targetOrderArray[i] * 2;
                    view_line(
                        dpplt->color,
                        dpat->xyArray[mP], dpat->xyArray[mP + 1],
                        dpat->xyArray[mT], dpat->xyArray[mT + 1],
                        _3D_LINE_SIZE, 0);
                }
                //
                dpplt = dpplt->next;
            }
        }
    }

    //注释
    dct = dpat->comment;
    while (dct)
    {
        //三维坐标转二维
        _3D_xyz_to_xy(dct->xyz, dct->xy);
        dct->xy[0] = centreX - dct->xy[0];
        dct->xy[1] = centreY - dct->xy[1];
        //
        view_string(dct->color, -1, dct->comment, dct->xy[0], dct->xy[1], 160, 0);
        //
        dct = dct->next;
    }
}
