
#include "tft_3d.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 0:增量式,适合连续变化的旋转(会累积误差)
// 1:每次都从原始坐标进行XYZ依次旋转,适合一次旋转
#define _3D_MODE_SWITCH  0

#define _3D_DRAW_PO_LINE  0    //point-origin connect
#define _3D_DRAW_PP_LINK  0    //point-point connect
#if(_3D_DRAW_PP_LINK)
#define _3D_DRAW_PP_LINK_LAST  0  //last point-first point connect
#endif

#define _3D_LINE_SIZE   1

#define _3D_PD_X   0.6
#define _3D_PD_Y   1.0
#define _3D_PD_Z   1.0

//
void _3D_reset(_3D_PointArray_Type *ddat)
{
    _3D_Comment_Type *dct;
    //
    memcpy(ddat->array, ddat->arrayCopy, ddat->memSize);
    memset(ddat->out, 0, ddat->pointNum*2*sizeof(int));
    memset(ddat->raxyz, 0, sizeof(ddat->raxyz));
    memset(ddat->mvxyz, 0, sizeof(ddat->mvxyz));
    //
    dct = ddat->comment;
    while(dct)
    {
        memcpy(dct->outXYZ, dct->outXYZCopy, sizeof(dct->outXYZ));
        memset(dct->outXY, 0, sizeof(dct->outXY));
        //
        dct = dct->next;
    }
}

//
void _3D_ppLink_add(_3D_PointArray_Type *ddat, int point, int targetNum, ...)
{
    int i, count, tempTarget;
    _3D_PPLink_Type *dpplt;
    va_list ap;
    //
    if(ddat == NULL || point >= ddat->pointNum)
        return;
    //还没开辟内存
    if(ddat->link == NULL)
    {
        dpplt = ddat->link = (_3D_PPLink_Type *)calloc(1, sizeof(_3D_PPLink_Type));
        dpplt = ddat->link;
    }
    else
    {
        dpplt = ddat->link;
        //移至末尾
        while(dpplt->next)
            dpplt = dpplt->next;
        //末尾新建
        dpplt->next = (_3D_PPLink_Type *)calloc(1, sizeof(_3D_PPLink_Type));
        dpplt = dpplt->next;
    }
    //
    va_start(ap , targetNum);
    dpplt->point = point;
    dpplt->target = (int *)calloc(targetNum + 1, sizeof(int));
    for(i = 0, count = 0; i < targetNum; i++)
    {
        tempTarget = va_arg(ap , int);
        if(tempTarget < ddat->pointNum)
        {
            dpplt->target[count] = tempTarget;
            count += 1;
        }
    }
    dpplt->targetNum = count;
}

//
void _3D_comment_add(_3D_PointArray_Type *ddat, double x, double y, double z, char *str, int color)
{
    _3D_Comment_Type *dct;
    //
    if(ddat == NULL || str == NULL)
        return;
    //
    if(ddat->comment == NULL)
    {
        ddat->comment = (_3D_Comment_Type *)calloc(1, sizeof(_3D_Comment_Type));
        dct = ddat->comment;
    }
    else
    {
        dct = ddat->comment;
        while(dct->next)
            dct = dct->next;
        dct->next = (_3D_Comment_Type *)calloc(1, sizeof(_3D_Comment_Type));
        dct = dct->next;
    }
    //
    dct->outXYZ[0] = dct->outXYZCopy[0] = x;
    dct->outXYZ[1] = dct->outXYZCopy[1] = y;
    dct->outXYZ[2] = dct->outXYZCopy[2] = z;
    dct->out = (char *)calloc(strlen(str)+16, sizeof(char));
    strcpy(dct->out, str);
    dct->color = color;
}

//
_3D_PointArray_Type *_3D_pointArray_init(int pointNum, ...)
{
    int i, j;
    _3D_PointArray_Type *ddat;
    va_list ap;
    //
    if(pointNum <= 0)
        return NULL;
    //
    ddat = (_3D_PointArray_Type *)calloc(1, sizeof(_3D_PointArray_Type));
    //
    ddat->pointNum = pointNum;
    ddat->memSize = pointNum*3*sizeof(double);
    //
    ddat->array = (double *)calloc(pointNum*3 + 3, sizeof(double));
    ddat->arrayCopy = (double *)calloc(pointNum*3 + 3, sizeof(double));
    //
    ddat->out = (int *)calloc(pointNum*2 + 2, sizeof(int));
    ddat->outColor = (int *)calloc(pointNum + 1, sizeof(int));
    //
    va_start(ap , pointNum);
    for(i = 0, j = 0; i < pointNum; i++)
    {
        ddat->array[j] = va_arg(ap , double);
        ddat->array[j+1] = va_arg(ap , double);
        ddat->array[j+2] = va_arg(ap , double);
        ddat->outColor[i] = va_arg(ap , int);
        // printf("input : %lf / %lf / %lf : %d\r\n", 
        //     ddat->array[j], 
        //     ddat->array[j+1], 
        //     ddat->array[j+2], 
        //     ddat->outColor[i]);
        j += 3;
    }
    va_end(ap);
    memcpy(ddat->arrayCopy, ddat->array, ddat->memSize);
    //
    return ddat;
}

//======================================================
//      将空间xyz坐标转换为平面xy坐标
//======================================================
void _3D_xyz_to_xy(double _3D_XYZ[3], int _2D_XY[2])
{
    double tempX, tempY;
    double x,y,z;
    //
    x = _3D_XYZ[0];
    y = _3D_XYZ[1];
    z = _3D_XYZ[2];
    //
    if(_3D_Type == 0)
    {
        // Y
        _2D_XY[0] = y*_3D_PD_Y;

        // Z
        _2D_XY[1] = z*_3D_PD_Z;

        // X
        if(x != 0)
        {
            tempX = x*_3D_PD_X;
            tempY = x*(1 - _3D_PD_X);
            //
            _2D_XY[0] -= (int)tempX;
            _2D_XY[1] -= (int)tempY;
        }
    }
}

//======================================================
//      将3轴转角转换为3组xyz坐标
//      (即根据 _3D_J* 转换 _3D_M*[3] 的坐标值)
//
//      X/Y/Zrad : 绕X/Y/Z轴的转角(rad)
//      point[3] : 要修正的空间向量的坐标
//======================================================
void _3D_angle_to_xyz0(double raxyz[3], double point[3])
{
    double x,y,z;
    double Xrad,Yrad,Zrad;

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
        x*cos(Yrad)*cos(Zrad) - 
        y*cos(Yrad)*sin(Zrad) + 
        z*sin(Yrad);
    point[1] = 
        x*(sin(Xrad)*sin(Yrad)*cos(Zrad) + cos(Xrad)*sin(Zrad)) - 
        y*(sin(Xrad)*sin(Yrad)*sin(Zrad) - cos(Xrad)*cos(Zrad)) - 
        z*sin(Xrad)*cos(Yrad);
    point[2] = 
        -x*(cos(Xrad)*sin(Yrad)*cos(Zrad) - sin(Xrad)*sin(Zrad)) + 
        y*(cos(Xrad)*sin(Yrad)*sin(Zrad) + sin(Xrad)*cos(Zrad)) + 
        z*cos(Xrad)*cos(Yrad);
}

void _3D_angle_to_xyz(_3D_PointArray_Type *ddat)       
{
    int i, j;
    _3D_Comment_Type *dct;
    //
    if(ddat == NULL || 
        ddat->array == NULL || 
        ddat->arrayCopy == NULL)
        return;
    //
    if(ddat->raxyz[0] >= 2*_3D_PI)
        ddat->raxyz[0] -= 2*_3D_PI;
    else if(ddat->raxyz[0] < 0)
        ddat->raxyz[0] += 2*_3D_PI;

    if(ddat->raxyz[1] >= 2*_3D_PI)
        ddat->raxyz[1] -= 2*_3D_PI;
    else if(ddat->raxyz[1] < 0)
        ddat->raxyz[1] += 2*_3D_PI;

    if(ddat->raxyz[2] >= 2*_3D_PI)
        ddat->raxyz[2] -= 2*_3D_PI;
    else if(ddat->raxyz[2] < 0)
        ddat->raxyz[2] += 2*_3D_PI;

    //
    for(i = 0, j = 0; i < ddat->pointNum; i++)
    {
#if(_3D_MODE_SWITCH)
        //mode/0: 使用原始的坐标和累积的转角量,一次转换到目标坐标
        memcpy(&ddat->array[j], &ddat->arrayCopy[j], 3*sizeof(double));
#endif
        //
        _3D_angle_to_xyz0(ddat->raxyz, &ddat->array[j]);
        //
        ddat->array[j] += ddat->mvxyz[0];
        ddat->array[j+1] += ddat->mvxyz[1];
        ddat->array[j+2] += ddat->mvxyz[2];
        //
        j += 3;
    }
    //
    dct = ddat->comment;
    while(dct)
    {
        _3D_angle_to_xyz0(ddat->raxyz, dct->outXYZ);
        //
        dct->outXYZ[0] += ddat->mvxyz[0];
        dct->outXYZ[1] += ddat->mvxyz[1];
        dct->outXYZ[2] += ddat->mvxyz[2];
        //
        dct = dct->next;
    }

#if(!_3D_MODE_SWITCH)
    //mode/1: 每次转换都使用的上次转换的坐标,转角量使用过后清零
    memset(ddat->raxyz, 0, sizeof(ddat->raxyz));
    memset(ddat->mvxyz, 0, sizeof(ddat->mvxyz));
#endif

}

//======================================================
//      绘制基准X\Y\Z轴 和 动态X\Y\Z轴
//
//      centreX/centreY : 设定原点在屏幕的坐标
//      m*[3] : 动态X\Y\Z轴的空间坐标
//======================================================
void _3D_draw(int centreX, int centreY, _3D_PointArray_Type *ddat)
{
    int i, j, k;
    _3D_PPLink_Type *dpplt;
    int mP, mT;
    _3D_Comment_Type *dct;
    //
    if(ddat == NULL)
        return;

    //三维坐标转二维
    for(i = 0, j = 0, k = 0; i < ddat->pointNum; i++)
    {
        _3D_xyz_to_xy(&ddat->array[j], &ddat->out[k]);

        ddat->out[k] = centreX - ddat->out[k];
        ddat->out[k+1] = centreY - ddat->out[k+1];
        // printf("2DXY: %d / %d\r\n", ddat->out[k], ddat->out[k+1]);
        view_dot(ddat->outColor[i], ddat->out[k], ddat->out[k+1], 2);
        //
        j += 3;
        k += 2;
    }

    //原点和各个点连线
#if(_3D_DRAW_PO_LINE)
    for(i = 0, j = 0; i < ddat->pointNum; i++)
    {
        view_line(ddat->outColor[i], 
            centreX, centreY, 
            ddat->out[j], ddat->out[j+1], 
            _3D_LINE_SIZE, 0);
        j += 2;
    }
#endif

    //点和点的连线
    if(ddat->pointNum > 1)
    {
#if(_3D_DRAW_PP_LINK)
        //前一点和下一点连线
        for(i = 0, j = 0; i < ddat->pointNum - 1; i++)
        {
            view_line(
                (ddat->outColor[i]+ddat->outColor[i+1])/2, 
                ddat->out[j], ddat->out[j+1], 
                ddat->out[j+2], ddat->out[j+3], 
                _3D_LINE_SIZE, 0);
            j += 2;
        }
#if(_3D_DRAW_PP_LINK_LAST)
        //最后一点和第一点连线
        view_line(
            (ddat->outColor[0]+ddat->outColor[ddat->pointNum-1])/2, 
            ddat->out[0], ddat->out[1], 
            ddat->out[(ddat->pointNum-1)*2], ddat->out[(ddat->pointNum-1)*2+1], 
            _3D_LINE_SIZE, 0);
#endif
#endif
        //根据ppLink关系连线
        if((dpplt = ddat->link))
        {
            while(dpplt)
            {
                mP = dpplt->point*2;
                for(i = 0; i < dpplt->targetNum; i++)
                {
                    mT = dpplt->target[i]*2;
                    view_line(
                        (ddat->outColor[dpplt->point]+ddat->outColor[dpplt->target[i]])/2, 
                        ddat->out[mP], ddat->out[mP+1], 
                        ddat->out[mT], ddat->out[mT+1], 
                        _3D_LINE_SIZE, 0); 
                }
                //
                dpplt = dpplt->next;
            }
        }
    }

    //注释
    dct = ddat->comment;
    while(dct)
    {
        //三维坐标转二维
        _3D_xyz_to_xy(dct->outXYZ, dct->outXY);
        dct->outXY[0] = centreX - dct->outXY[0];
        dct->outXY[1] = centreY - dct->outXY[1];
        //
        view_string(dct->color, -1, dct->out, dct->outXY[0], dct->outXY[1], 160, 0);
        //
        dct = dct->next;
    }
}

