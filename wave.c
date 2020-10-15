/*
 *  自制简易版示波器,图像输出到fb0
 */
#include <stdlib.h>
#include <string.h>

#include "fbmap.h"

#define WAVE_CHN 6
#define WAVE_Y_OFFSET 320

static FbMap *fbmap;

static int wave_width = 100;
static int wave_height = 100;
static int wave_height_half = 50;
static int wave_data_size = 100 * 100 * 3;

static short *wave_chn[WAVE_CHN];
static unsigned char *wave_data;
static int wave_refresh_count = 0;

const char wave_color[WAVE_CHN][3] = {
    {0xFF, 0x00, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0x00, 0xFF},
    {0xFF, 0xFF, 0x00},
    {0x00, 0xFF, 0xFF},
    {0xFF, 0x00, 0xFF},
};

static void wav_init()
{
    int i;
    if (fbmap)
        return;

    fbmap = fb_init(0, WAVE_Y_OFFSET);
    if (!fbmap)
        return;
    wave_width = fbmap->fbInfo.xres;
    wave_height = fbmap->fbInfo.yres - WAVE_Y_OFFSET;
    wave_height_half = wave_height / 2;
    wave_data_size = wave_width * wave_height * 3;

    wave_data = (unsigned char *)calloc(wave_data_size, sizeof(char));
    for (i = 0; i < WAVE_CHN; i++)
        wave_chn[i] = (short *)calloc(wave_width, sizeof(short));
}

void wave_line(int xStart, int yStart, int xEnd, int yEnd, char *rgb)
{
    int offset;
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
        offset = (yCount * wave_width + xCount) * 3;
        wave_data[offset + 0] = (wave_data[offset + 0] + rgb[0]) >> 1;
        wave_data[offset + 1] = (wave_data[offset + 1] + rgb[1]) >> 1;
        wave_data[offset + 2] = (wave_data[offset + 2] + rgb[2]) >> 1;

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

/*
 *  chn: 0~5
 */
void wave_load(int chn, short value)
{
    if (chn < 0 || chn >= WAVE_CHN)
        return;

    wav_init();
    if (!fbmap)
        return;

    wave_chn[chn][wave_refresh_count] = value;
}

void wave_refresh(void)
{
    int i, j, x, y, ox = 0, oy = 0;

    wav_init();
    if (!fbmap)
        return;

    memset(wave_data, 0, wave_data_size);

    // base line
    memset(wave_data + wave_height_half * wave_width * 3, 0xFF, wave_width * 3);

    // chn line
    for (i = 0; i < WAVE_CHN; i++)
    {
        ox = oy = 0;
        for (j = 0; j <= wave_refresh_count; j++)
        {
            y = wave_height_half - wave_chn[i][j] * wave_height_half / 32768;
            if (y < 0)
                y = 0;
            else if (y >= wave_height)
                y = wave_height - 1;
            x = j;

            wave_line(ox, oy, x, y, (char *)wave_color[i]);

            ox = x;
            oy = y;
        }
    }

    // draw
    fb_refresh(fbmap, wave_data, wave_width, wave_height, 3);

    wave_refresh_count += 1;
    if (wave_refresh_count >= wave_width)
    {
        for (i = 0; i < WAVE_CHN; i++)
            for (j = 0; j < wave_width - 1; j++)
                wave_chn[i][j] = wave_chn[i][j + 1];

        wave_refresh_count -= 1;
    }
}
