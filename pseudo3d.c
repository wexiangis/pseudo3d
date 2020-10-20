
/*
 *  自制乞丐版3D引擎
 */
#include "pseudo3d.h"
#include "view.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//传入旋转角度的矩阵模式: 0/使用xyz旋转矩阵 1/使用zyx旋转矩阵
#define P3D_MATRIX_MODE 1

//传入旋转和平移数值模式: 0/固定值 1/增量值
#define P3D_INPUT_MODE 0

//三维坐标轴的原点和三维图形的各个顶点连线
#define P3D_AUTO_LINK_PO 0 //point-origin connect

//自动把输入的各个点依次连线
#define P3D_AUTO_LINK_PP 0 //point-point connect
#if (P3D_AUTO_LINK_PP)
//自动把最后一个点和第一个点连线
#define P3D_AUTO_LINK_PP_LAST 0 //last point-first point connect
#endif

//画线的线宽
#define P3D_LINE_SIZE 1

//建坐标的方式  0: x/ y| z-
#define P3D_2D_XYZ_TYPE 0

//三维投影二维时各轴向长度的缩放比例
#define P3D_2D_X 0.6
#define P3D_2D_Y 1.0
#define P3D_2D_Z 1.0

/*
 *  把空间坐标point[3]转换为物体自身坐标系
 *  参数:
 *      raxyz[3] : 绕X/Y/Z轴的转角(rad: 0~2pi)
 *      point[3] : 要修正的空间向量的坐标,输出值回写到这里面
 */
void p3d_matrix_xyz(double raxyz[3], double point[3])
{
    double x = point[0], y = point[1], z = point[2];
    double A = raxyz[0], B = raxyz[1], C = raxyz[2];
#if 0
    /*
    *       [roll X]
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
    *                                   |x|
    *  result = [roll X][roll Y][roll Z]|y|
    *                                   |z|
    *
    *           |cB,     0,  sB    |        |x|
    *         = |sB*sA,  cA, -cB*sA|[roll Z]|y|
    *           |-sB*cA, sA, cB*cA |        |z|
    * 
    *           |cC*cB,             -sC*cB,            sB    ||x|
    *         = |cC*sB*sA + sC*cA,  -sC*sB*sA + cC*cA, -cB*sA||x|
    *           |-cC*sB*cA + sC*sA, sC*sB*cA + cC*sA,  cB*cA ||z|
    *
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */

    point[0] =
        x * cos(C) * cos(B) +
        y * (-sin(C) * cos(B)) +
        z * sin(B);
    point[1] =
        x * (cos(C) * sin(B) * sin(A) + sin(C) * cos(A)) +
        y * (-sin(C) * sin(B) * sin(A) + cos(C) * cos(A)) +
        z * (-cos(B) * sin(A));
    point[2] =
        x * (-cos(C) * sin(B) * cos(A) + sin(C) * sin(A)) +
        y * (sin(C) * sin(B) * cos(A) + cos(C) * sin(A)) +
        z * cos(B) * cos(A);
#else
    /*
    *       [roll X]
    *   1       0       0
    *   0     cosA     sinA
    *   0    -sinA     cosA
    *
    *       [roll Y]
    *  cosB     0     -sinB
    *   0       1       0
    *  sinB     0      cosB
    *
    *       [roll Z]
    *  cosC    sinC     0
    * -sinC    cosC     0
    *   0       0       1
    *
    *                                   |x|
    *  result = [roll X][roll Y][roll Z]|y|
    *                                   |z|
    *
    *           |cB,    0,   -sB  |        |x|
    *         = |sB*sA, cA,  cB*sA|[roll Z]|y|
    *           |sB*cA, -sA, cB*cA|        |z|
    * 
    *           |cC*cB,            sC*cB,            -sB  ||x|
    *         = |cC*sB*sA - sC*cA, sC*sB*sA + cC*cA, cB*sA||x|
    *           |cC*sB*cA + sC*sA, sC*sB*cA - cC*sA, cB*cA||z|
    *
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */

    point[0] =
        x * cos(C) * cos(B) +
        y * sin(C) * cos(B) +
        z * (-sin(B));
    point[1] =
        x * (cos(C) * sin(B) * sin(A) - sin(C) * cos(A)) +
        y * (sin(C) * sin(B) * sin(A) + cos(C) * cos(A)) +
        z * cos(B) * sin(A);
    point[2] =
        x * (cos(C) * sin(B) * cos(A) + sin(C) * sin(A)) +
        y * (sin(C) * sin(B) * cos(A) - cos(C) * sin(A)) +
        z * cos(B) * cos(A);
#endif
}

/*
 *  把物体自身坐标point[3]转换为空间坐标
 *  参数:
 *      raxyz[3] : 绕X/Y/Z轴的转角(rad: 0~2pi)
 *      point[3] : 要修正的空间向量的坐标,输出值回写到这里面
 */
void p3d_matrix_zyx(double raxyz[3], double point[3])
{
    double x = point[0], y = point[1], z = point[2];
    double A = raxyz[0], B = raxyz[1], C = raxyz[2];
#if 1
    /*
    *       [roll Z]
    *  cosC   -sinC     0
    *  sinC    cosC     0
    *   0       0       1
    *
    *       [roll Y]
    *  cosB     0      sinB
    *   0       1       0
    * -sinB     0      cosB
    * 
    *       [roll X]
    *   1       0       0
    *   0     cosA    -sinA
    *   0     sinA     cosA
    *
    *                                   |x|
    *  result = [roll Z][roll Y][roll X]|y|
    *                                   |z|
    *
    *           |cB*cC, -sC, sB*cC|        |x|
    *         = |cB*sC, cC,  sB*sC|[roll X]|y|
    *           |-sB,   0,   cB   |        |z|
    * 
    *           |cB*cC, -cA*sC + sA*sB*cC, sA*sC + cA*sB*cC ||x|
    *         = |cB*sC, cA*cC + sA*sB*sC,  -sA*cC + cA*sB*sC||x|
    *           |-sB,   sA*cB,             cA*cB            ||z|
    *
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */

    point[0] =
        x * cos(B) * cos(C) +
        y * (-cos(A) * sin(C) + sin(A) * sin(B) * cos(C)) +
        z * (sin(A) * sin(C) + cos(A) * sin(B) * cos(C));
    point[1] =
        x * cos(B) * sin(C) +
        y * (cos(A) * cos(C) + sin(A) * sin(B) * sin(C)) +
        z * (-sin(A) * cos(C) + cos(A) * sin(B) * sin(C));
    point[2] =
        x * (-sin(B)) +
        y * sin(A) * cos(B) +
        z * cos(A) * cos(B);
#else
    /*
    *       [roll Z]
    *  cosC    sinC     0
    * -sinC    cosC     0
    *   0       0       1
    *
    *       [roll Y]
    *  cosB     0     -sinB
    *   0       1       0
    *  sinB     0      cosB
    * 
    *       [roll X]
    *   1       0       0
    *   0     cosA     sinA
    *   0    -sinA     cosA
    *
    *                                   |x|
    *  result = [roll Z][roll Y][roll X]|y|
    *                                   |z|
    *
    *           |cB*cC,  sC, -sB*cC|        |x|
    *         = |-cB*sC, cC, sB*sC |[roll X]|y|
    *           |sB,     0,  cB    |        |z|
    * 
    *           |cB*cC,  cA*sC + sA*sB*cC, sA*sC - cA*sB*cC||x|
    *         = |-cB*sC, cA*cC - sA*sB*sC, sA*cC + cA*sB*sC||x|
    *           |sB,     -sA*cB,           cA*cB           ||z|
    *
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */

    point[0] =
        x * cos(B) * cos(C) +
        y * (cos(A) * sin(C) + sin(A) * sin(B) * cos(C)) +
        z * (sin(A) * sin(C) - cos(A) * sin(B) * cos(C));
    point[1] =
        x * (-cos(B) * sin(C)) +
        y * (cos(A) * cos(C) - sin(A) * sin(B) * sin(C)) +
        z * (sin(A) * cos(C) + cos(A) * sin(B) * sin(C));
    point[2] =
        x * sin(B) +
        y * (-sin(A) * cos(B)) +
        z * cos(A) * cos(B);
#endif
}

/*
 *  重置三维图形到初始化时的状态
 */
void p3d_reset(P3D_PointArray_Type *dpat)
{
    P3D_Comment_Type *dct;

    memcpy(dpat->xyzArray, dpat->xyzArrayCopy, dpat->xyzArrayMemSize);
    memset(dpat->xyArray, 0, dpat->pointNum * 2 * sizeof(int));
    memset(dpat->raxyz, 0, sizeof(dpat->raxyz));
    memset(dpat->mvxyz, 0, sizeof(dpat->mvxyz));

    dct = dpat->comment;
    while (dct)
    {
        memcpy(dct->xyz, dct->xyzCopy, sizeof(dct->xyz));
        memset(dct->xy, 0, sizeof(dct->xy));

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
void p3d_ppLink_add(P3D_PointArray_Type *dpat, int color, int point, int targetNum, ...)
{
    int i, count, tempTarget;
    P3D_PPLink_Type *dpplt;
    va_list ap;

    if (dpat == NULL || point >= dpat->pointNum)
        return;

    //还没开辟内存
    if (dpat->link == NULL)
    {
        dpplt = dpat->link = (P3D_PPLink_Type *)calloc(1, sizeof(P3D_PPLink_Type));
        dpplt = dpat->link;
    }
    else
    {
        dpplt = dpat->link;
        //移至末尾
        while (dpplt->next)
            dpplt = dpplt->next;
        //末尾新建
        dpplt->next = (P3D_PPLink_Type *)calloc(1, sizeof(P3D_PPLink_Type));
        dpplt = dpplt->next;
    }

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
void p3d_comment_add(P3D_PointArray_Type *dpat, double x, double y, double z, char *comment, int type, int color)
{
    P3D_Comment_Type *dct;

    if (dpat == NULL || comment == NULL)
        return;

    if (dpat->comment == NULL)
    {
        dpat->comment = (P3D_Comment_Type *)calloc(1, sizeof(P3D_Comment_Type));
        dct = dpat->comment;
    }
    else
    {
        dct = dpat->comment;
        while (dct->next)
            dct = dct->next;
        dct->next = (P3D_Comment_Type *)calloc(1, sizeof(P3D_Comment_Type));
        dct = dct->next;
    }

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
P3D_PointArray_Type *p3d_init(int pointNum, double x, double y, double z, int color, ...)
{
    int i, j;
    P3D_PointArray_Type *dpat;
    va_list ap;

    if (pointNum < 1)
        return NULL;

    dpat = (P3D_PointArray_Type *)calloc(1, sizeof(P3D_PointArray_Type));

    dpat->pointNum = pointNum;
    dpat->xyzArrayMemSize = pointNum * 3 * sizeof(double);

    dpat->xyzArray = (double *)calloc(pointNum * 3 + 3, sizeof(double));
    dpat->xyzArrayCopy = (double *)calloc(pointNum * 3 + 3, sizeof(double));

    dpat->xyArray = (int *)calloc(pointNum * 2 + 2, sizeof(int));
    dpat->color = (int *)calloc(pointNum + 1, sizeof(int));

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

    dpat->_matrix_mode = P3D_MATRIX_MODE;
    return dpat;
}

/*
 *  把三维坐标点投影到二维坐标点上
 */
void p3d_3d_to_2d(double P3D_XYZ[3], int _2D_XY[2])
{
    double tempX, tempY;
    double x = P3D_XYZ[0], y = P3D_XYZ[1], z = P3D_XYZ[2];

    //不同的坐标系摆放方式
    if (P3D_2D_XYZ_TYPE == 0)
    {
        // Y
        _2D_XY[0] = y * P3D_2D_Y;
        // Z
        _2D_XY[1] = z * P3D_2D_Z;
        // X
        if (x != 0)
        {
            tempX = x * P3D_2D_X;
            tempY = x * (1 - P3D_2D_X);
            //
            _2D_XY[0] -= (int)tempX;
            _2D_XY[1] -= (int)tempY;
        }
    }
}

/*
 *  根据当前 dpat 中的 raxyz[3] 和 mvxyz[3] 对当前的图形进行旋转和平移
 */
void p3d_refresh(P3D_PointArray_Type *dpat)
{
    int i, j;
    P3D_Comment_Type *dct;

    if (dpat == NULL ||
        dpat->xyzArray == NULL ||
        dpat->xyzArrayCopy == NULL)
        return;

    //限制角度范围[0, 2pi]
    if (dpat->raxyz[0] >= P3D_2PI)
        dpat->raxyz[0] -= P3D_2PI;
    else if (dpat->raxyz[0] < 0)
        dpat->raxyz[0] += P3D_2PI;

    if (dpat->raxyz[1] >= P3D_2PI)
        dpat->raxyz[1] -= P3D_2PI;
    else if (dpat->raxyz[1] < 0)
        dpat->raxyz[1] += P3D_2PI;

    if (dpat->raxyz[2] >= P3D_2PI)
        dpat->raxyz[2] -= P3D_2PI;
    else if (dpat->raxyz[2] < 0)
        dpat->raxyz[2] += P3D_2PI;
    //计算各点位置
    for (i = 0, j = 0; i < dpat->pointNum; i++)
    {
#if (P3D_INPUT_MODE != 1)
        //mode/0: 使用原始的坐标和累积的转角量,一次转换到目标坐标
        memcpy(&dpat->xyzArray[j], &dpat->xyzArrayCopy[j], 3 * sizeof(double));
#endif
        //用旋转角度 raxyz[3] 处理3维坐标点 xyzArray[3]
        if(dpat->_matrix_mode == 1)
            p3d_matrix_zyx(dpat->raxyz, &dpat->xyzArray[j]);
        else
            p3d_matrix_xyz(dpat->raxyz, &dpat->xyzArray[j]);
        //平移量(相对于绝对坐标系)
        dpat->xyzArray[j] += dpat->mvxyz[0];
        dpat->xyzArray[j + 1] += dpat->mvxyz[1];
        dpat->xyzArray[j + 2] += dpat->mvxyz[2];
        //下一个点
        j += 3;
    }
    //计算注释位置
    dct = dpat->comment;
    while (dct)
    {
#if (P3D_INPUT_MODE != 1)
        //mode/0: 使用原始的坐标和累积的转角量,一次转换到目标坐标
        memcpy(dct->xyz, dct->xyzCopy, 3 * sizeof(double));
#endif
        //用旋转角度 raxyz[3] 处理3维坐标点 xyz[3]
        if(dpat->_matrix_mode == 1)
            p3d_matrix_zyx(dpat->raxyz, dct->xyz);
        else
            p3d_matrix_xyz(dpat->raxyz, dct->xyz);
        //平移量(相对于绝对坐标系)
        dct->xyz[0] += dpat->mvxyz[0];
        dct->xyz[1] += dpat->mvxyz[1];
        dct->xyz[2] += dpat->mvxyz[2];
        //下一条注释
        dct = dct->next;
    }
#if (P3D_INPUT_MODE == 1)
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
void p3d_draw(int centreX, int centreY, P3D_PointArray_Type *dpat)
{
    int i, j, k;
    P3D_PPLink_Type *dpplt;
    int mP, mT;
    P3D_Comment_Type *dct;

    if (dpat == NULL)
        return;

    //运算
    p3d_refresh(dpat);

    //三维坐标转二维
    for (i = 0, j = 0, k = 0; i < dpat->pointNum; i++)
    {
        p3d_3d_to_2d(&dpat->xyzArray[j], &dpat->xyArray[k]);
        dpat->xyArray[k] = centreX - dpat->xyArray[k];
        dpat->xyArray[k + 1] = centreY - dpat->xyArray[k + 1];
        //输出
        // printf("2DXY: %d / %d\r\n", dpat->xyArray[k], dpat->xyArray[k+1]);
        view_dot(dpat->color[i], dpat->xyArray[k], dpat->xyArray[k + 1], 2);

        j += 3;
        k += 2;
    }

    //原点和各个点连线
#if (P3D_AUTO_LINK_PO)
    for (i = 0, j = 0; i < dpat->pointNum; i++)
    {
        view_line(dpat->color[i],
                  centreX, centreY,
                  dpat->xyArray[j], dpat->xyArray[j + 1],
                  P3D_LINE_SIZE, 0);
        j += 2;
    }
#endif

    //点和点的连线
    if (dpat->pointNum > 1)
    {
#if (P3D_AUTO_LINK_PP)
        //前一点和下一点连线
        for (i = 0, j = 0; i < dpat->pointNum - 1; i++)
        {
            view_line(
                (dpat->color[i] + dpat->color[i + 1]) / 2,
                dpat->xyArray[j], dpat->xyArray[j + 1],
                dpat->xyArray[j + 2], dpat->xyArray[j + 3],
                P3D_LINE_SIZE, 0);
            j += 2;
        }
#if (P3D_AUTO_LINK_PP_LAST)
        //最后一点和第一点连线
        view_line(
            (dpat->color[0] + dpat->color[dpat->pointNum - 1]) / 2,
            dpat->xyArray[0], dpat->xyArray[1],
            dpat->xyArray[(dpat->pointNum - 1) * 2], dpat->xyArray[(dpat->pointNum - 1) * 2 + 1],
            P3D_LINE_SIZE, 0);
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
                        P3D_LINE_SIZE, 0);
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
        p3d_3d_to_2d(dct->xyz, dct->xy);
        dct->xy[0] = centreX - dct->xy[0];
        dct->xy[1] = centreY - dct->xy[1];
        //输出
        view_string(dct->color, -1, dct->comment, dct->xy[0], dct->xy[1], 160, 0);

        dct = dct->next;
    }
}

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
    double *xyz)
{
    int xy[2];
    //坐标转换
    p3d_3d_to_2d(xyz, xy);
    //偏移到中心点
    xy[0] = centreX - xy[0];
    xy[1] = centreY - xy[1];
    //输出
    view_dot(color, xy[0], xy[1], 2);
    view_line(color, centreX, centreY, xy[0], xy[1], 1, 0);
}
