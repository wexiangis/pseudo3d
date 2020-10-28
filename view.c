
/*
 *  图像输出到fb0
 */
#include "view.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//---------------------------------------------------------------------------------------------------------------------------

#ifdef VIEW_ENABLE_FB
#include "fbmap.h"
static FbMap *fbmap = NULL;
#endif

static unsigned char amoled_data[VIEW_Y_SIZE][VIEW_X_SIZE][VIEW_PICTURE_PERW];

void amoled_print_dot(int x, int y, const unsigned char *data)
{
    int i;
    if (x < 0 || y < 0 || x >= VIEW_X_SIZE || y >= VIEW_Y_SIZE)
        return;
    for (i = 0; i < VIEW_PICTURE_PERW; i++)
        amoled_data[y][x][i] = data[i];
}

void amoled_print_dot2(int x, int y, const unsigned char *data)
{
    int i;
    if (x < 0 || y < 0 || x >= VIEW_X_SIZE || y >= VIEW_Y_SIZE)
        return;
    for (i = 0; i < VIEW_PICTURE_PERW; i++)
        amoled_data[y][x][i] = (data[i] + amoled_data[y][x][i]) / 2;
}

void amoled_print_en(void)
{
#ifdef VIEW_ENABLE_FB
    if (!fbmap) {
        fbmap = fb_init(0, 0);
        if(fbmap)
            fbmap->xOffset = fbmap->fbInfo.xres - VIEW_X_SIZE - 1;
    }
    fb_refresh(fbmap, (unsigned char *)amoled_data, VIEW_X_SIZE, VIEW_Y_SIZE, VIEW_PICTURE_PERW);
#else
    bmp_create("./test.bmp", (unsigned char *)amoled_data, VIEW_X_SIZE, VIEW_Y_SIZE, VIEW_PICTURE_PERW);
#endif
}

void amoled_print_clear(void)
{
    memset(amoled_data, 0, VIEW_Y_SIZE * VIEW_X_SIZE * VIEW_PICTURE_PERW);
}

//---------------------------------------------------------------------------------------------------------------------------示波器

//---------------------------------------------------------------------------------------------------------------------------

void view_delay_us(unsigned int us)
{
    struct timeval delay;
    if (us > 1000000)
    {
        delay.tv_sec = us / 1000000;
        delay.tv_usec = us % 1000000; //us延时
    }
    else
    {
        delay.tv_sec = 0;
        delay.tv_usec = us; //us延时
    }
    select(0, NULL, NULL, NULL, &delay);
}

//---------------------------------------------------------------------------------------------------------------------------点, 圆, 环

//功能: 画圆或圆环
//参数: color : 颜色
//          xStart :
//          yStart :
//          rad : 半径(外经)
//          size : 半径向里画环的圈数  0:完全填充, >0: 画环
//返回: 无
void view_circle(long color, int xStart, int yStart, int rad, int size)
{
    int circle_a, circle_b;
    int circle_di;
    int circle_rad = rad;
    int circle_size = size;
    int i;
    long colorTemp = color;
    unsigned char col[8];
    //
    if (circle_rad <= 0)
        return;
    // 颜色解析
    for (i = 0; i < VIEW_PICTURE_PERW && i < sizeof(col); i++)
        col[i] = (unsigned char)((colorTemp >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
    //
    if (circle_size <= 0)
        circle_size = rad;
    //
    for (; circle_rad > rad - circle_size && circle_rad > 0; circle_rad--)
    {
        circle_a = 0;
        circle_b = circle_rad;
        circle_di = 3 - (circle_rad << 1);
        while (circle_a <= circle_b)
        {
            //1/4
            PRINT_DOT(xStart + circle_a, yStart - circle_b, col);     //1
            PRINT_DOT(xStart + circle_a, yStart - circle_b + 1, col); //补点
            PRINT_DOT(xStart + circle_b, yStart - circle_a, col);     //2
            PRINT_DOT(xStart + circle_b - 1, yStart - circle_a, col); //补点
            //2/4
            PRINT_DOT(xStart + circle_b, yStart + circle_a, col);     //3
            PRINT_DOT(xStart + circle_b - 1, yStart + circle_a, col); //补点
            PRINT_DOT(xStart + circle_a, yStart + circle_b, col);     //4
            PRINT_DOT(xStart + circle_a, yStart + circle_b - 1, col); //补点
            //3/4
            PRINT_DOT(xStart - circle_a, yStart + circle_b, col);     //5
            PRINT_DOT(xStart - circle_a, yStart + circle_b - 1, col); //补点
            PRINT_DOT(xStart - circle_b, yStart + circle_a, col);     //6
            PRINT_DOT(xStart - circle_b + 1, yStart + circle_a, col); //补点
            //4/4
            PRINT_DOT(xStart - circle_b, yStart - circle_a, col);     //7
            PRINT_DOT(xStart - circle_b + 1, yStart - circle_a, col); //补点
            PRINT_DOT(xStart - circle_a, yStart - circle_b, col);     //8
            PRINT_DOT(xStart - circle_a, yStart - circle_b + 1, col); //补点
            //
            circle_a++;
            //使用Bresenham算法画圆
            if (circle_di < 0)
                circle_di += 4 * circle_a + 6;
            else
            {
                circle_di += 10 + 4 * (circle_a - circle_b);
                circle_b--;
            }
        }
    }
}
//功能: 画圆环, 扇形, 扇形圆环
//参数: color : 颜色
//          xStart :
//          yStart :
//          rad : 半径(外经)
//          size : 半径向里画环的圈数  0:完全填充, >0: 画环
//          div : 把圆拆分多少分  0或1 : 画整圆, >1: 拆分多份(此时 divStart, divEnd 参数有效)
//          divStart, divEnd : 只画 divStart ~ divEnd 的圆环
//返回: 无
//说明:
void view_circleLoop(long color, int xStart, int yStart, int rad, int size, int div, int divStart, int divEnd)
{
    int circle_a, circle_b;
    int circle_di;
    int circle_rad = rad;
    int circle_size = size;
    //
    int angleCount, arrayCount;
    float rangeArray[8][2];
    float divStartTemp, divEndTemp;
    //
    int **intArray = NULL, sumCount, sumCount2;
    //
    int i;
    long colorTemp = color;
    unsigned char col[8];
    //
    if (div < 0 || (div > 1 &&
                    (divStart < 1 || divStart > div || divEnd < 1 || divEnd > div || divEnd < divStart)))
        return;
    if (circle_rad <= 0)
        return;
    if (circle_size <= 0)
        circle_size = rad;
    // 颜色解析
    for (i = 0; i < VIEW_PICTURE_PERW && i < sizeof(col); i++)
        col[i] = (unsigned char)((colorTemp >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
    //
    if (div > 1)
    {
        divStartTemp = (divStart - 1) * 360.00 / div; //开始度数
        divEndTemp = divEnd * 360.00 / div;           //结束度数
        //用来记录一圈原始数据的缓冲区
        intArray = (int **)calloc(rad + 1, sizeof(int *));
        for (i = 0; i < rad + 1; i++)
            intArray[i] = (int *)calloc(2, sizeof(int));
    }
    //
    for (; circle_rad > rad - circle_size && circle_rad > 0; circle_rad--)
    {
        circle_a = 0;
        circle_b = circle_rad;
        circle_di = 3 - (circle_rad << 1);
        //
        sumCount = 0;
        while (circle_a <= circle_b)
        {
            if (div < 2) //div < 2 的都是画完整一圈的
            {
                PRINT_DOT(xStart + circle_a, yStart - circle_b, col); //1
                PRINT_DOT(xStart + circle_b, yStart - circle_a, col); //2
                PRINT_DOT(xStart + circle_b, yStart + circle_a, col); //3
                PRINT_DOT(xStart + circle_a, yStart + circle_b, col); //4
                PRINT_DOT(xStart - circle_a, yStart + circle_b, col); //5
                PRINT_DOT(xStart - circle_b, yStart + circle_a, col); //6
                PRINT_DOT(xStart - circle_b, yStart - circle_a, col); //7
                PRINT_DOT(xStart - circle_a, yStart - circle_b, col); //8
                //if(circle_size > 1)
                {
                    PRINT_DOT(xStart + circle_a, yStart - circle_b + 1, col); //补点
                    PRINT_DOT(xStart + circle_b - 1, yStart - circle_a, col); //补点
                    PRINT_DOT(xStart + circle_b - 1, yStart + circle_a, col); //补点
                    PRINT_DOT(xStart + circle_a, yStart + circle_b - 1, col); //补点
                    PRINT_DOT(xStart - circle_a, yStart + circle_b - 1, col); //补点
                    PRINT_DOT(xStart - circle_b + 1, yStart + circle_a, col); //补点
                    PRINT_DOT(xStart - circle_b + 1, yStart - circle_a, col); //补点
                    PRINT_DOT(xStart - circle_a, yStart - circle_b + 1, col); //补点
                }
            }
            else //先记下一圈数据, 后期处理后再画
            {
                intArray[sumCount][0] = circle_a;
                intArray[sumCount][1] = circle_b;
                sumCount += 1;
            }
            //
            circle_a++;
            //使用Bresenham算法画圆
            if (circle_di < 0)
                circle_di += 4 * circle_a + 6;
            else
            {
                circle_di += 10 + 4 * (circle_a - circle_b);
                circle_b--;
            }
        }
        //要分段画扇形的在此处理
        if (div > 1)
        {
            //计算范围数组 rangeArray[][]
            for (i = 0, angleCount = 45, arrayCount = 0; i < 8;)
            {
                //
                if (divStartTemp >= angleCount || divEndTemp <= angleCount - 45)
                {
                    rangeArray[arrayCount][0] = 999;
                    rangeArray[arrayCount][1] = -1;
                }
                else
                {
                    if (divStartTemp <= angleCount - 45)
                        rangeArray[arrayCount][0] = -1;
                    else
                        rangeArray[arrayCount][0] = (divStartTemp - (angleCount - 45)) * sumCount / 45 - 1;
                    //
                    if (divEndTemp >= angleCount)
                        rangeArray[arrayCount][1] = 999;
                    else
                        rangeArray[arrayCount][1] = (divEndTemp - (angleCount - 45)) * sumCount / 45 + 1;
                }
                //
                angleCount += 45;
                arrayCount += 1;
                i += 1;
            }
            //绘制一圈
            for (i = 0, sumCount2 = sumCount - 1; i < sumCount; i++)
            {
                circle_a = intArray[i][0];
                circle_b = intArray[i][1];
                //
                if (circle_a > rangeArray[0][0] && circle_a < rangeArray[0][1])
                {
                    PRINT_DOT(xStart + circle_a, yStart - circle_b, col);     //1
                    PRINT_DOT(xStart + circle_a, yStart - circle_b + 1, col); //补点
                }
                if (sumCount2 - circle_a > rangeArray[1][0] && sumCount2 - circle_a < rangeArray[1][1])
                {
                    PRINT_DOT(xStart + circle_b, yStart - circle_a, col);     //2
                    PRINT_DOT(xStart + circle_b - 1, yStart - circle_a, col); //补点
                }
                if (circle_a > rangeArray[2][0] && circle_a < rangeArray[2][1])
                {
                    PRINT_DOT(xStart + circle_b, yStart + circle_a, col);     //3
                    PRINT_DOT(xStart + circle_b - 1, yStart + circle_a, col); //补点
                }
                if (sumCount2 - circle_a > rangeArray[3][0] && sumCount2 - circle_a < rangeArray[3][1])
                {
                    PRINT_DOT(xStart + circle_a, yStart + circle_b, col);     //4
                    PRINT_DOT(xStart + circle_a, yStart + circle_b - 1, col); //补点
                }
                if (circle_a > rangeArray[4][0] && circle_a < rangeArray[4][1])
                {
                    PRINT_DOT(xStart - circle_a, yStart + circle_b, col);     //5
                    PRINT_DOT(xStart - circle_a, yStart + circle_b - 1, col); //补点
                }
                if (sumCount2 - circle_a > rangeArray[5][0] && sumCount2 - circle_a < rangeArray[5][1])
                {
                    PRINT_DOT(xStart - circle_b, yStart + circle_a, col);     //6
                    PRINT_DOT(xStart - circle_b + 1, yStart + circle_a, col); //补点
                }
                if (circle_a > rangeArray[6][0] && circle_a < rangeArray[6][1])
                {
                    PRINT_DOT(xStart - circle_b, yStart - circle_a, col);     //7
                    PRINT_DOT(xStart - circle_b + 1, yStart - circle_a, col); //补点
                }
                if (sumCount2 - circle_a > rangeArray[7][0] && sumCount2 - circle_a < rangeArray[7][1])
                {
                    PRINT_DOT(xStart - circle_a, yStart - circle_b, col);     //8
                    PRINT_DOT(xStart - circle_a, yStart - circle_b + 1, col); //补点
                }
            }
        }
    }
    //记得释放内存
    if (div > 1)
    {
        for (i = 0; i < rad + 1; i++)
            free(intArray[i]);
        free(intArray);
    }
}

//功能: 画点函数
//参数: color : 颜色
//          xStart :
//          yStart :
//          size : 1~2
//返回: 无
void view_dot(long color, int xStart, int yStart, int size)
{
    unsigned char col[8];
    long colorTemp = color;
    int i;
    //
    for (i = 0; i < VIEW_PICTURE_PERW && i < sizeof(col); i++)
        col[i] = (unsigned char)((colorTemp >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
    //mid
    PRINT_DOT(xStart, yStart, col);
    //
    if (size == 2)
    {
        //up
        PRINT_DOT(xStart, yStart + 1, col);
        //down
        PRINT_DOT(xStart, yStart - 1, col);
        //left
        PRINT_DOT(xStart + 1, yStart, col);
        //right
        PRINT_DOT(xStart - 1, yStart, col);
    }
    else if (size > 2)
        view_circle(color, xStart, yStart, size, 0);
}

//---------------------------------------------------------------------------------------------------------------------------线

//功能: 指定起止坐标, 返回两点间画线的Y坐标数组
//参数: xStart, yStart, xEnd, yEnd : 起止坐标
//          dot : Y坐标数组起始地址, 需要自己先分配好内存再传入
//返回: 无
void view_getDotFromLine(int xStart, int yStart, int xEnd, int yEnd, int *dot)
{
    unsigned short t;
    int xerr = 0, yerr = 0;
    int delta_x, delta_y;
    int distance;
    int incx, incy, xCount, yCount;

    delta_x = xEnd - xStart; //计算坐标增量
    delta_y = yEnd - yStart;
    xCount = xStart;
    yCount = yStart;
    //
    if (delta_x > 0)
        incx = 1; //设置单步方向
    else if (delta_x == 0)
        incx = 0; //垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    //
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; //水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    //
    if (delta_x > delta_y)
        distance = delta_x; //选取基本增量坐标轴
    else
        distance = delta_y;
    //
    for (t = 0; t <= distance + 1; t++) //画线输出
    {
        *dot++ = yCount;

        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            xCount += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            yCount += incy;
        }
    }
}

//功能: 划线函数
//参数: xStart, yStart, xEnd, yEnd : 起止坐标
//          size : 线宽
//          space : 不为0时画的是虚线, 其值代表虚线的点密度
//返回: 无
void view_line(long color, int xStart, int yStart, int xEnd, int yEnd, int size, int space)
{
    unsigned short t;
    int xerr = 0, yerr = 0;
    int delta_x, delta_y;
    int distance;
    int incx, incy, xCount, yCount;
    int spaceCount = 0, spaceVal = 0;

    if (size <= 0)
        return;

    delta_x = xEnd - xStart; //计算坐标增量
    delta_y = yEnd - yStart;
    xCount = xStart;
    yCount = yStart;
    //
    if (delta_x > 0)
        incx = 1; //设置单步方向
    else if (delta_x == 0)
        incx = 0; //垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    //
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; //水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    //
    if (delta_x > delta_y)
        distance = delta_x; //选取基本增量坐标轴
    else
        distance = delta_y;
    //
    if (space == 0)
        spaceVal = 0;
    else if (space > 0)
    {
        spaceVal = space;
        spaceCount = -space;
    }
    else
    {
        spaceVal = -space;
        spaceCount = 0;
    }
    //
    for (t = 0; t <= distance + 1; t++) //画线输出
    {
        if (spaceVal == 0 || spaceCount < 0)
        {
            spaceCount += 1;
            if (size == 1)
                view_dot(color, xCount, yCount, 1); //画点
            else
                view_dot(color, xCount, yCount, 2); //画点
        }
        else
        {
            spaceCount += 1;
            if (spaceCount >= spaceVal)
                spaceCount = -spaceVal;
        }

        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            xCount += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            yCount += incy;
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------矩形

//功能: 画矩形
//参数: color : 颜色
//          xStart, yStart, xEnd, yEnd : 起止坐标
//          size : 线宽
//          rad : 圆角半径
//          mode : 0:实心填充  1:半透填充
//          minY, maxY : 超出上下 Y 坐标部分不绘制
//返回: 无
void view_rectangle(long color, int xStart, int yStart, int xEnd, int yEnd, int size, int rad, int mode, int minX, int minY, int maxX, int maxY) //0:按照size画框  1:实心填充  2:半透填充
{
    int xS = xStart, yS = yStart, xE = xEnd, yE = yEnd;
    int xSize, ySize, sSize = size;
    int xC, yC, temp;
    int i, j, k;
    long colorTemp = color;
    unsigned char col[8];
    //
    int circle_rad = rad;
    int circle_a, circle_b;
    int circle_di;
    int circle_localCentre[4][2]; //4个角的圆心坐标
    //
    int tempArray[4][2]; //4个圆角的临时变量
    char **outPutArray;
    //
    if (circle_rad < 0)
        circle_rad = 0;
    if (sSize < 0)
        sSize = 0;
    // 矩阵端点整理
    if (xS > xE && yS <= yE) // 交换x坐标
    {
        temp = xS;
        xS = xE;
        xE = temp;
    }
    else if (yS > yE && xS <= xE) // 交换y坐标
    {
        temp = yS;
        yS = yE;
        yE = temp;
    }
    else if (yS > yE && xS > xE) // 交换x, y坐标
    {
        temp = xS;
        xS = xE;
        xE = temp;
        temp = yS;
        yS = yE;
        yE = temp;
    }
    xSize = xE - xS + 1;
    ySize = yE - yS + 1;
    if (xS > maxX || xE < minX || yS > maxY || yE < minY || xSize < 1 || ySize < 1)
        return;
    if (sSize && (sSize * 2 > xSize || sSize * 2 > ySize))
        sSize = 0;
    // 颜色解析
    for (i = 0; i < VIEW_PICTURE_PERW && i < sizeof(col); i++)
        col[i] = (unsigned char)((colorTemp >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
    //缓存数组初始化
    outPutArray = (char **)calloc(ySize, sizeof(char *));
    for (i = 0; i < ySize; i++)
        outPutArray[i] = (char *)calloc(xSize, sizeof(char));
    //中间横
    if (sSize > 0)
    {
        for (i = circle_rad; i < ySize - circle_rad; i++)
        {
            for (j = 0; j < sSize; j++)
                outPutArray[i][j] = 1;
            for (j = xSize - sSize; j < xSize; j++)
                outPutArray[i][j] = 1;
        }
    }
    else
    {
        for (i = circle_rad; i < ySize - circle_rad; i++)
        {
            for (j = 0; j < xSize; j++)
                outPutArray[i][j] = 1;
        }
    }
    //
    if (sSize > 0)
        temp = sSize;
    else
        temp = circle_rad;
    //
    if (sSize > 0)
    {
        //上横
        for (i = 0; i < temp; i++)
        {
            for (j = circle_rad; j < xSize - circle_rad; j++)
                outPutArray[i][j] = 1;
        }
        //下横
        for (i = ySize - temp; i < ySize; i++)
        {
            for (j = circle_rad; j < xSize - circle_rad; j++)
                outPutArray[i][j] = 1;
        }
    }
    //
    if (circle_rad > 0)
    {
        //圆角必须小于长宽
        if (xSize / 2 < circle_rad)
            circle_rad = xSize / 2;
        if (ySize / 2 < circle_rad)
            circle_rad = ySize / 2;
        //
        circle_localCentre[0][0] = circle_rad;
        circle_localCentre[0][1] = circle_rad; //左上角圆心
        circle_localCentre[1][0] = xSize - circle_rad - 1;
        circle_localCentre[1][1] = circle_rad; //右上角圆心
        circle_localCentre[2][0] = circle_rad;
        circle_localCentre[2][1] = ySize - circle_rad - 1; //左下角圆心
        circle_localCentre[3][0] = xSize - circle_rad - 1;
        circle_localCentre[3][1] = ySize - circle_rad - 1; //右下角圆心
        //
        if (sSize == 0)
        {
            temp = 1;
            tempArray[0][1] = -1;
            tempArray[1][1] = circle_rad;
            tempArray[2][1] = ySize - circle_rad - 1;
            tempArray[3][1] = 9999;
        }
        //
        for (i = 0; i < temp && circle_rad > 0; i++, circle_rad--)
        {
            circle_a = 0;
            circle_b = circle_rad;
            circle_di = 3 - (circle_rad << 1);
            while (circle_a <= circle_b)
            {
                if (sSize)
                {
                    //1/4
                    outPutArray[circle_localCentre[1][1] - circle_b][circle_localCentre[1][0] + circle_a] = 1;
                    outPutArray[circle_localCentre[1][1] - circle_a][circle_localCentre[1][0] + circle_b] = 1;
                    //2/4
                    outPutArray[circle_localCentre[3][1] + circle_a][circle_localCentre[3][0] + circle_b] = 1;
                    outPutArray[circle_localCentre[3][1] + circle_b][circle_localCentre[3][0] + circle_a] = 1;
                    //3/4
                    outPutArray[circle_localCentre[2][1] + circle_b][circle_localCentre[2][0] - circle_a] = 1;
                    outPutArray[circle_localCentre[2][1] + circle_a][circle_localCentre[2][0] - circle_b] = 1;
                    //4/4
                    outPutArray[circle_localCentre[0][1] - circle_a][circle_localCentre[0][0] - circle_b] = 1;
                    outPutArray[circle_localCentre[0][1] - circle_b][circle_localCentre[0][0] - circle_a] = 1;
                    //
                    if (sSize > 1)
                    {
                        //1/4
                        outPutArray[circle_localCentre[1][1] - circle_b + 1][circle_localCentre[1][0] + circle_a] = 1;
                        outPutArray[circle_localCentre[1][1] - circle_a][circle_localCentre[1][0] + circle_b - 1] = 1;
                        //2/4
                        outPutArray[circle_localCentre[3][1] + circle_a][circle_localCentre[3][0] + circle_b - 1] = 1;
                        outPutArray[circle_localCentre[3][1] + circle_b - 1][circle_localCentre[3][0] + circle_a] = 1;
                        //3/4
                        outPutArray[circle_localCentre[2][1] + circle_b - 1][circle_localCentre[2][0] - circle_a] = 1;
                        outPutArray[circle_localCentre[2][1] + circle_a + 1][circle_localCentre[2][0] - circle_b] = 1;
                        //4/4
                        outPutArray[circle_localCentre[0][1] - circle_a + 1][circle_localCentre[0][0] - circle_b] = 1;
                        outPutArray[circle_localCentre[0][1] - circle_b + 1][circle_localCentre[0][0] - circle_a] = 1;
                    }
                }
                else
                {
                    if (circle_localCentre[1][1] - circle_b > tempArray[0][1]) //1, 8  y
                    {
                        tempArray[0][1] = circle_localCentre[1][1] - circle_b;
                        //
                        xC = circle_localCentre[0][0] - circle_a; //8 x
                        k = circle_localCentre[1][0] + circle_a;  //1 x
                        yC = tempArray[0][1];
                        for (; xC < k; xC++)
                            outPutArray[yC][xC] = 1;
                    }
                    if (circle_localCentre[1][1] - circle_a < tempArray[1][1]) //2, 7  y
                    {
                        tempArray[1][1] = circle_localCentre[1][1] - circle_a;
                        //
                        xC = circle_localCentre[0][0] - circle_b; //7 x
                        k = circle_localCentre[1][0] + circle_b;  //2 x
                        yC = tempArray[1][1];
                        for (; xC < k; xC++)
                            outPutArray[yC][xC] = 1;
                    }
                    if (circle_localCentre[3][1] + circle_a > tempArray[2][1]) //3, 6  y
                    {
                        tempArray[2][1] = circle_localCentre[3][1] + circle_a;
                        //
                        xC = circle_localCentre[2][0] - circle_b; //6 x
                        k = circle_localCentre[3][0] + circle_b;  //3 x
                        yC = tempArray[2][1];
                        for (; xC <= k; xC++)
                            outPutArray[yC][xC] = 1;
                    }
                    if (circle_localCentre[3][1] + circle_b < tempArray[3][1]) //4, 5  y
                    {
                        tempArray[3][1] = circle_localCentre[3][1] + circle_b;
                        //
                        xC = circle_localCentre[2][0] - circle_a; //5 x
                        k = circle_localCentre[3][0] + circle_a;  //4 x
                        yC = tempArray[3][1];
                        for (; xC < k; xC++)
                            outPutArray[yC][xC] = 1;
                    }
                }
                //
                circle_a++;
                //使用Bresenham算法画圆
                if (circle_di < 0)
                    circle_di += 4 * circle_a + 6;
                else
                {
                    circle_di += 10 + 4 * (circle_a - circle_b);
                    circle_b--;
                }
            }
        }
    }
    //输出
    for (i = 0, yC = yS; i < ySize; i++, yC++)
    {
        if (yC < minY || yC > maxY)
            continue;
        if (mode == 1)
        {
            for (j = 0, xC = xS; j < xSize; j++, xC++)
            {
                if (xC < minX || xC > maxX)
                    continue;
                else if (outPutArray[i][j])
                    PRINT_DOT2(xC, yC, col);
            }
        }
        else
        {
            for (j = 0, xC = xS; j < xSize; j++, xC++)
            {
                if (xC < minX || xC > maxX)
                    continue;
                else if (outPutArray[i][j])
                    PRINT_DOT(xC, yC, col);
            }
        }
    }
    //
    for (i = 0; i < ySize; i++)
        free(outPutArray[i]);
    free(outPutArray);
}

//功能: 画矩形并填充图片
//参数: pic : 图片数据
//          xStart, yStart, xEnd, yEnd : 起止坐标
//返回: 无
void view_rectangle_padding(const unsigned char *pic, int xStart, int yStart, int xEnd, int yEnd)
{
    int xS = xStart, yS = yStart, xE = xEnd, yE = yEnd;
    int xC, yC, temp;
    int i;
    // 矩阵端点整理
    if (xS > xE && yS <= yE) // 交换x坐标
    {
        temp = xS;
        xS = xE;
        xE = temp;
    }
    else if (yS > yE && xS <= xE) // 交换y坐标
    {
        temp = yS;
        yS = yE;
        yE = temp;
    }
    else if (yS > yE && xS > xE) // 交换x, y坐标
    {
        temp = xS;
        xS = xE;
        xE = temp;
        temp = yS;
        yS = yE;
        yE = temp;
    }
    // 写入缓冲区
    for (i = 0, yC = yS; yC <= yE; yC++)
    {
        for (xC = xS; xC <= xE; xC++)
        {
            PRINT_DOT(xC, yC, &pic[i]);
            i += VIEW_PICTURE_PERW;
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------平行四边形

//功能: 画平行四边形
//参数: color : 颜色
//          xStart, yStart, xEnd, yEnd : 起止坐标   //平行四边形 左上 和 右下 的坐标
//          size : 线宽
//          width : 平行四边形上边长度
//          mode : 0:实心填充  1:半透填充
//          minY, maxY : 超出上下 Y 坐标部分不绘制
//返回: 无
void view_parallelogram(long color, int xStart, int yStart, int xEnd, int yEnd, int size, int width, int mode, int minX, int minY, int maxX, int maxY) //0:实心填充  1:半透填充
{
    int xS = xStart, yS = yStart, xE = xEnd, yE = yEnd;
    int xC, yC, temp, sSize = size;
    ;
    int i;
    long colorTemp = color;
    unsigned char col[8];
    //
    int xEnd2, yEnd2;
    unsigned short t;
    int xerr = 0, yerr = 0;
    int delta_x, delta_y;
    int distance;
    int incx, incy, xCount, yCount;
    // 矩阵端点整理
    if (xS > xE && yS <= yE) // 交换x坐标
    {
        temp = xS;
        xS = xE;
        xE = temp;
    }
    else if (yS > yE && xS <= xE) // 交换y坐标
    {
        temp = yS;
        yS = yE;
        yE = temp;
    }
    else if (yS > yE && xS > xE) // 交换x, y坐标
    {
        temp = xS;
        xS = xE;
        xE = temp;
        temp = yS;
        yS = yE;
        yE = temp;
    }
    // 颜色解析
    for (i = 0; i < VIEW_PICTURE_PERW && i < sizeof(col); i++)
        col[i] = (unsigned char)((colorTemp >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
    //
    if (sSize < 0)
        sSize = 0;
    //
    xEnd2 = xEnd - width;
    yEnd2 = yEnd;
    //
    delta_x = xEnd2 - xStart; //计算坐标增量
    delta_y = yEnd2 - yStart;
    xCount = xStart;
    yCount = yStart;
    //
    if (delta_x > 0)
        incx = 1; //设置单步方向
    else if (delta_x == 0)
        incx = 0; //垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    //
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; //水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    //
    if (delta_x > delta_y)
        distance = delta_x; //选取基本增量坐标轴
    else
        distance = delta_y;
    //
    for (t = 0; t <= distance + 1; t++) //画线输出
    {
        // xCount, yCount
        if (yCount < minY || yCount > maxY)
            ;
        else
        {
            if (mode == 1)
            {
                for (xC = xCount, yC = yCount; xC <= xCount + width; xC++)
                {
                    if (xC < minX || xC > maxX)
                        continue;
                    if (sSize == 0)
                        PRINT_DOT2(xC, yC, col);
                    else if (yC < yStart + sSize || xC < xCount + sSize || yC > yEnd2 - sSize || xC > xCount + width - sSize)
                        PRINT_DOT2(xC, yC, col);
                }
            }
            else
            {
                for (xC = xCount, yC = yCount; xC <= xCount + width; xC++)
                {
                    if (xC < minX || xC > maxX)
                        continue;
                    if (sSize == 0)
                        PRINT_DOT(xC, yC, col);
                    else if (yC < yStart + sSize || xC < xCount + sSize || yC > yEnd2 - sSize || xC > xCount + width - sSize)
                        PRINT_DOT(xC, yC, col);
                }
            }
        }
        //
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            xCount += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            yCount += incy;
        }
    }
    // xCount, yCount
    if (yCount < minY || yCount > maxY)
        ;
    else
    {
        if (mode == 1)
        {
            for (xC = xEnd2, yC = yEnd2; xC <= xCount + width; xC++)
            {
                if (xC < minX || xC > maxX)
                    continue;
                if (sSize == 0)
                    PRINT_DOT2(xC, yC, col);
                else if (yC < yStart + sSize || xC < xCount + sSize || yC > yEnd2 - sSize || xC > xCount + width - sSize)
                    PRINT_DOT2(xC, yC, col);
            }
        }
        else
        {
            for (xC = xEnd2, yC = yEnd2; xC <= xCount + width; xC++)
            {
                if (xC < minX || xC > maxX)
                    continue;
                if (sSize == 0)
                    PRINT_DOT(xC, yC, col);
                else if (yC < yStart + sSize || xC < xCount + sSize || yC > yEnd2 - sSize || xC > xCount + width - sSize)
                    PRINT_DOT(xC, yC, col);
            }
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------写字符串

//功能: 字符矩阵输出
//参数: fColor : 打印颜色
//          bColor : 背景颜色, -1 时使用原图填充(也就是透明)
//          buf : 矩阵数据
//          bufLen : buf的字节数
//          width : 矩阵宽度
//          xStart, yStart : 矩阵的左上角定位坐标
//返回: 无
void view_string_print(unsigned char *fColor, unsigned char *bColor, unsigned char *buf, unsigned int bufLen, int width, int xStart, int yStart)
{
    int i, j;
    unsigned char dotOfChar;
    int xC = xStart, yC = yStart;

    for (i = 0; i < bufLen; i++)
    {
        dotOfChar = buf[i];
        for (j = 0; j < 8; j++)
        {
            if ((dotOfChar & 0x80) == 0x80 && fColor)
                PRINT_DOT(xC++, yC, fColor);
            else if (bColor == NULL) // bColor < 0 表示设置背景透明
                xC++;
            else
                PRINT_DOT(xC++, yC, bColor);
            dotOfChar <<= 1;
            if (xC - xStart >= width) //横向满48个点换行
            {
                xC = xStart;
                yC += 1;
            }
        }
    }
}

//功能: 字符串输出
//参数: fColor : 打印颜色
//          bColor : 背景颜色, -1 时使用原图填充(也就是透明)
//          str : 字符串
//          xStart, yStart : 矩阵的左上角定位坐标
//          type : 字体, 例如 160, 240, 320, 400, 480, 560, 640, 前两位标识像素尺寸, 后1位表示字体
//          space : 字符间隔, 正常输出为0
//返回: 无
void view_string(long fColor, long bColor, char *str, int xStart, int yStart, int type, int space)
{
    int i = 0;
    int ret;
    unsigned char buf[512] = {0};
    unsigned int bufLen = 0;
    int retWidth;
    unsigned char fCol[8], bCol[8], *fp, *bp;
    //
    if (fColor >= 0)
    {
        for (i = 0; i < VIEW_PICTURE_PERW && i < 8; i++)
            fCol[i] = (unsigned char)((fColor >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
        fp = fCol;
    }
    else
        fp = NULL;
    if (bColor >= 0)
    {
        for (i = 0; i < VIEW_PICTURE_PERW && i < 8; i++)
            bCol[i] = (unsigned char)((bColor >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
        bp = bCol;
    }
    else
        bp = NULL;
    //
    i = 0;
    while (*str)
    {
        memset(buf, 0, sizeof(buf));
        ret = gbk_getArrayByUtf8((unsigned char *)str, buf, &bufLen, type);
        if (ret < 0)
            str += (-ret);
        else
        {
            retWidth = bufLen * 8 / (type / 10);
            //printf("%s ret = %d, bufLen = %d, retWidth = %d\r\n", str, ret, bufLen, retWidth);
            view_string_print(fp, bp, buf, bufLen, retWidth, xStart + i, yStart);
            i += (retWidth + space);
            str += ret;
        }
    }
}

//功能: 字符矩阵输出, 带范围限制
//参数: fColor : 打印颜色
//          bColor : 背景颜色, -1 时使用原图填充(也就是透明)
//          buf : 矩阵数据
//          bufLen : buf的字节数
//          width : 矩阵宽度
//          xStart, yStart : 矩阵的左上角定位坐标
//          xScreenStart, yScreenStart, xScreenEnd, yScreenEnd : 限制范围, 超出部分不绘制
//返回: 无
void view_string_print2(unsigned char *fColor, unsigned char *bColor,
                        unsigned char *buf, unsigned int bufLen, int width, int xStart, int yStart, int xScreenStart, int yScreenStart, int xScreenEnd, int yScreenEnd, int printMode)
{
    int i, j;
    unsigned char dotOfChar;
    int xC = xStart, yC = yStart;

    for (i = 0; i < bufLen; i++)
    {
        dotOfChar = buf[i];
        for (j = 0; j < 8; j++)
        {
            if (xC >= xScreenStart && xC < xScreenEnd && yC >= yScreenStart && yC < yScreenEnd)
            {
                if ((dotOfChar & 0x80) == 0x80 && fColor)
                {
                    if (printMode)
                        PRINT_DOT2(xC++, yC, fColor);
                    else
                        PRINT_DOT(xC++, yC, fColor);
                }
                else if (bColor == NULL) // bColor < 0 表示设置背景透明
                    xC++;
                else
                    PRINT_DOT(xC++, yC, bColor);
            }
            else
                xC++;
            dotOfChar <<= 1;
            if (xC - xStart >= width) //横向满48个点换行
            {
                xC = xStart;
                yC += 1;
            }
        }
    }
}

//功能: 字符串输出, 带范围限制
//参数: fColor : 打印颜色
//          bColor : 背景颜色, -1 时使用原图填充(也就是透明)
//          str : 字符串
//          xStart, yStart : 矩阵的左上角定位坐标
//          strWidth, strHight : 相对左上角定位坐标, 限制宽, 高的矩阵内输出字符串
//          type : 字体, 例如 160, 240, 320, 400, 480, 560, 640, 前两位标识像素尺寸, 后1位表示字体
//          space : 字符间隔, 正常输出为0
//返回: 无
void view_string_rectangle(long fColor, long bColor, char *str, int xStart, int yStart, int strWidth, int strHight, int xScreenStart, int yScreenStart, int xScreenEnd, int yScreenEnd, int type, int space, int printMode)
{
    int i = 0;
    int ret;
    unsigned char buf[512] = {0};
    unsigned int bufLen = 0;
    int retWidth;
    unsigned char fCol[8], bCol[8], *fp, *bp;
    //
    if (fColor >= 0)
    {
        for (i = 0; i < VIEW_PICTURE_PERW && i < 8; i++)
            fCol[i] = (unsigned char)((fColor >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
        fp = fCol;
    }
    else
        fp = NULL;
    if (bColor >= 0)
    {
        for (i = 0; i < VIEW_PICTURE_PERW && i < 8; i++)
            bCol[i] = (unsigned char)((bColor >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
        bp = bCol;
    }
    else
        bp = NULL;
    //
    i = 0;
    while (*str)
    {
        memset(buf, 0, sizeof(buf));
        ret = gbk_getArrayByUtf8((unsigned char *)str, buf, &bufLen, type);
        if (ret < 0)
            str += (-ret);
        else
        {
            retWidth = bufLen * 8 / (type / 10);
            //printf("%s ret = %d, bufLen = %d, retWidth = %d\r\n", str, ret, bufLen, retWidth);
            view_string_print2(fp, bp, buf, bufLen, retWidth, xStart + i, yStart, xScreenStart, yScreenStart, xScreenEnd, yScreenEnd, printMode);
            i += (retWidth + space);
            str += ret;
        }
    }
}

//功能: 字符串输出, 带范围限制, 加滚动
//参数: fColor : 打印颜色
//          bColor : 背景颜色, -1 时使用原图填充(也就是透明)
//          str : 字符串
//          xStart, yStart : 矩阵的左上角定位坐标
//          strWidth, strHight : 相对左上角定位坐标, 限制宽, 高的矩阵内输出字符串
//          type : 字体, 例如 160, 240, 320, 400, 480, 560, 640, 前两位标识像素尺寸, 后1位表示字体
//          space : 字符间隔, 正常输出为0
//          xErr : 相对 xStart 坐标, 字符串输出前先按xErr的 负/正的量 进行 左/右偏移一定像素
//返回: 无
int view_string_rectangleCR(long fColor, long bColor, char *str, int xStart, int yStart, int strWidth, int strHight, int xScreenStart, int yScreenStart, int xScreenEnd, int yScreenEnd, int type, int space, int xErr, int printMode)
{
    int movX = xErr;
    int ret, strCount = 0, retVal = xErr;
    unsigned char buf[512] = {0};
    unsigned int bufLen = 0;
    int i, retWidth;
    unsigned char fCol[8], bCol[8], *fp, *bp;
    //
    if (str == NULL)
        return 0;
    //
    if (fColor >= 0)
    {
        for (i = 0; i < VIEW_PICTURE_PERW && i < 8; i++)
            fCol[i] = (unsigned char)((fColor >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
        fp = fCol;
    }
    else
        fp = NULL;
    if (bColor >= 0)
    {
        for (i = 0; i < VIEW_PICTURE_PERW && i < 8; i++)
            bCol[i] = (unsigned char)((bColor >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
        bp = bCol;
    }
    else
        bp = NULL;
    //
    while (movX < strWidth)
    {
        memset(buf, 0, sizeof(buf));
        ret = gbk_getArrayByUtf8((unsigned char *)&str[strCount], buf, &bufLen, type); //为ascii字符时ret=1, 为中文时ret=3
        if (ret < 0)
            strCount += (-ret);
        else
        {
            retWidth = bufLen * 8 / (type / 10);
            //printf("%s ret = %d, bufLen = %d, retWidth = %d\r\n", str, ret, bufLen, retWidth);
            if (movX + retWidth > 0)
                view_string_print2(fp, bp, buf, bufLen, retWidth, xStart + movX, yStart, xScreenStart, yScreenStart, xScreenEnd, yScreenEnd, printMode);
            movX += (retWidth + space);
            strCount += ret; //到当前为止总的字节数    //为ascii字符时ret=1, 为中文时ret=3
        }
        //
        if (str[strCount] == '\0') //字符串内容循环输出
        {
            //自动填充空格分开字符串
            movX += (type / 10 + space);
            //
            if (movX <= 0)     //字符串遍历完还未抵达开始绘制的位置
                retVal = movX; //返回此次绘制的偏差值, 以便后续无缝衔接
            strCount = 0;
        }
    }
    //
    return retVal;
}

//功能: 多行, 字符串输出, 带范围限制
//参数: fColor : 打印颜色
//          bColor : 背景颜色, -1 时使用原图填充(也就是透明)
//          str : 字符串
//          *xStart, *yStart : 矩阵的左上角定位坐标, "数组"
//          *strWidth, *strHight : 相对左上角定位坐标, 限制宽, 高的矩阵内输出字符串, "数组"
//          type : 字体, 例如 160, 240, 320, 400, 480, 560, 640, 前两位标识像素尺寸, 后1位表示字体
//          space : 字符间隔, 正常输出为0
//          lineNum : 行数
//          xErr : 相对 xStart 坐标, 字符串输出前先按xErr的 负/正的量 进行 左/右偏移一定像素
//          *retLineCharNum : 返回每行实际填充的字节数, "数组"
//返回: 无
int view_string_rectangleMultiLine(long fColor, long bColor, char *str, int *xStart, int *yStart, int *strWidth, int *strHight, int type, int space, int lineNum, int *retLineCharNum, int printMode)
{
    int i, ret, retWidth, retVal = 0;
    int movX = 0, strCount = 0;
    unsigned char buf[512] = {0};
    unsigned int bufLen = 0;
    unsigned char fCol[8], bCol[8], *fp, *bp;
    int lineCount = 0, lineStrCount = 0;
    int typeSize = type / 10;
    //
    if (str == NULL)
        return 0;
    //int型的颜色信息转成3字节数组
    if (fColor >= 0)
    {
        for (i = 0; i < VIEW_PICTURE_PERW && i < 8; i++)
            fCol[i] = (unsigned char)((fColor >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
        fp = fCol;
    }
    else
        fp = NULL;
    if (bColor >= 0)
    {
        for (i = 0; i < VIEW_PICTURE_PERW && i < 8; i++)
            bCol[i] = (unsigned char)((bColor >> ((VIEW_PICTURE_PERW - i - 1) * 8)) & 0xFF);
        bp = bCol;
    }
    else
        bp = NULL;
    //
    while (str[strCount])
    {
        if ((unsigned char)(str[strCount]) < 0x20) //特殊字符检测
        {
            lineStrCount += 1;
            //
            if (str[strCount] == 0x0A) //回车
                retWidth = strWidth[lineCount] + 1;
            else if (str[strCount] == 0x09) //Tab
                retWidth = typeSize + typeSize;
            else
                retWidth = typeSize;
            //
            if (movX + retWidth > strWidth[lineCount])
            {
                if (retLineCharNum)
                    retLineCharNum[lineCount] = lineStrCount;
                movX = 0;
                lineStrCount = 0;
                lineCount += 1;
                if (lineCount >= lineNum)
                    break;
            }
            else
                movX += (retWidth + space);
            //
            ret = -1;
        }
        else //正常字符 通过调库获得像素数据
        {
            memset(buf, 0, sizeof(buf));
            ret = gbk_getArrayByUtf8((unsigned char *)&str[strCount], buf, &bufLen, type); //为ascii字符时ret=1, 为中文时ret=3
        }
        //
        if (ret < 0)
            strCount += (-ret);
        else
        {
            retWidth = bufLen * 8 / typeSize;
            //printf("%s ret = %d, bufLen = %d, retWidth = %d\r\n", str, ret, bufLen, retWidth);
            if (movX + retWidth > strWidth[lineCount])
            {
                //如果需要, 返回每行实际填充的字节数(由于ascii和中文前者占1字节后者3字节, 这里每行占用的字节数不能用行宽度除以字节宽度来计算!!!)
                if (retLineCharNum)
                    retLineCharNum[lineCount] = lineStrCount;
                //行起始偏移量清零
                movX = 0;
                lineStrCount = 0;
                //行数加+1
                lineCount += 1;
                if (lineCount >= lineNum)
                    break;
            }
            view_string_print2(fp, bp, buf, bufLen, retWidth, xStart[lineCount] + movX, yStart[lineCount], xStart[lineCount], yStart[lineCount], xStart[lineCount] + strWidth[lineCount], yStart[lineCount] + strHight[lineCount], printMode);
            //
            movX += (retWidth + space); //行起始偏移量 = 字符实际宽度 + 使用的空格像素
            strCount += ret;            //到当前为止总的字节数    //为ascii字符时ret=1, 为中文时ret=3
            lineStrCount += ret;        //每行字节数的备份
        }
    }
    if (retLineCharNum)
        retLineCharNum[lineCount] = lineStrCount;
    retVal = strCount;
    //
    return retVal;
}
