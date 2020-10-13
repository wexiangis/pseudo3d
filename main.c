
/*
 *  通过mpu6050计算姿态信息，并输出成3D图像显示
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "pseudo3D.h"
#include "view.h"
#include "posture.h"
#include "wave.h"
#include "delayus.h"

#define INTERVALUS 50000

#define SCROLL_DIV (_3D_PI / 16)
#define MOVE_DIV 10

int main(int argc, char **argv)
{
    //初始化一个多边形
    _3D_PointArray_Type *dpat0, *dpat1;

    char input[16];
    int fd;

    // open console
    if(argc > 1)
        fd = open(argv[1], O_RDONLY);
    else
        fd = open("/dev/console", O_RDONLY);
    //非阻塞设置
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    //XYZ
    if ((dpat0 = _3D_pointArray_init(6,
        _3D_XYZ_ScanLen * 1.00, 0.00, 0.00, 0x800000,
        -_3D_XYZ_ScanLen * 1.00, 0.00, 0.00, 0x800000,
        0.00, _3D_XYZ_ScanLen * 1.00, 0.00, 0x008080,
        0.00, -_3D_XYZ_ScanLen * 1.00, 0.00, 0x008080,
        0.00, 0.00, _3D_XYZ_ScanLen * 1.00, 0x008000,
        0.00, 0.00, -_3D_XYZ_ScanLen * 1.00, 0x008000)) == NULL)
    {
        printf("_3D_pointArray_init failed\r\n");
        return -1;
    }
    _3D_ppLink_add(dpat0, 0x800000, 0, 1, 1);
    _3D_ppLink_add(dpat0, 0x008000, 2, 1, 3);
    _3D_ppLink_add(dpat0, 0x008080, 4, 1, 5);
    _3D_comment_add(dpat0, _3D_XYZ_ScanLen, 0, 0, "X", 0, 0x800000);
    _3D_comment_add(dpat0, 0, _3D_XYZ_ScanLen, 0, "Y", 0, 0x008080);
    _3D_comment_add(dpat0, 0, 0, _3D_XYZ_ScanLen, "Z", 0, 0x008000);

    //长方体
    if ((dpat1 = _3D_pointArray_init(8,
        40.00, 30.00, 50.00, 0xFF00FF,
        40.00, -30.00, 50.00, 0xFFFF00,
        -40.00, -30.00, 50.00, 0x00FFFF,
        -40.00, 30.00, 50.00, 0xFF8000,
        -40.00, 30.00, -50.00, 0xFF00FF,
        -40.00, -30.00, -50.00, 0xFFFF00,
        40.00, -30.00, -50.00, 0x00FFFF,
        40.00, 30.00, -50.00, 0xFF8000)) == NULL)
    {
        printf("_3D_pointArray_init failed\r\n");
        return -1;
    }
    _3D_ppLink_add(dpat1, 0xFF0000, 0, 3, 1, 3, 7);
    _3D_ppLink_add(dpat1, 0x00FF00, 1, 2, 2, 6);
    _3D_ppLink_add(dpat1, 0x0000FF, 2, 2, 3, 5);
    _3D_ppLink_add(dpat1, 0xFFFF00, 3, 1, 4);
    _3D_ppLink_add(dpat1, 0xFF00FF, 4, 2, 5, 7);
    _3D_ppLink_add(dpat1, 0x00FFFF, 5, 1, 6);
    _3D_ppLink_add(dpat1, 0xFF8000, 6, 1, 7);
    _3D_comment_add(dpat1, 40.00, 30.00, 50.00, "A", 0, 0xFFFF00);
    _3D_comment_add(dpat1, 40.00, -30.00, 50.00, "B", 0, 0x00FF00);
    _3D_comment_add(dpat1, -40.00, -30.00, 50.00, "C", 0, 0x8080FF);
    _3D_comment_add(dpat1, -40.00, 30.00, 50.00, "D", 0, 0xFF0000);
    _3D_comment_add(dpat1, -40.00, 30.00, -50.00, "E", 0, 0xFF00FF);
    _3D_comment_add(dpat1, -40.00, -30.00, -50.00, "F", 0, 0x00FFFF);
    _3D_comment_add(dpat1, 40.00, -30.00, -50.00, "G", 0, 0xFF8000);
    _3D_comment_add(dpat1, 40.00, 30.00, -50.00, "H", 0, 0x0080FF);

    //初始转角
    // dpat1->raxyz[0] = _3D_PI/8;
    // dpat1->raxyz[1] = _3D_PI/8;
    // dpat1->raxyz[2] = _3D_PI/8;

    //初始化姿态计算器
    posture_init(INTERVALUS / 1000);

    while (1)
    {
        wave_load(0, posture_getGyroX() * 5);
        wave_load(1, posture_getGyroY() * 5);
        wave_load(2, posture_getGyroZ() * 5);
        wave_load(3, posture_getAccelX());
        wave_load(4, posture_getAccelY());
        wave_load(5, posture_getAccelZ());

        dpat1->raxyz[0] = posture_getACX();
        dpat1->raxyz[1] = posture_getACY();
        dpat1->raxyz[2] = posture_getACZ();

        printf("x/%04d y/%04d z/%04d -- x/%04d y/%04d z/%04d -- x/%.4f y/%.4f z/%.4f\r\n",
            posture_getGyroX(), posture_getGyroY(), posture_getGyroZ(),
            posture_getAccelX(), posture_getAccelY(), posture_getAccelZ(),
            posture_getX(), posture_getY(), posture_getZ());

        PRINT_CLEAR();

        // memcpy(dpat2->raxyz, dpat1->raxyz, sizeof(dpat1->raxyz));

        // memcpy(dpat2->mvxyz, dpat1->mvxyz, sizeof(dpat1->mvxyz));

        // _3D_angle_to_xyz(dpat2);
        _3D_angle_to_xyz(dpat1);

        _3D_draw(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 2, dpat0);

        // _3D_draw(VIEW_X_SIZE/2, VIEW_Y_SIZE/2, dpat2);
        _3D_draw(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 2, dpat1);

        PRINT_EN();
        wave_refresh();

        // dpat1->raxyz[0] += SCROLL_DIV;
        // dpat1->raxyz[1] += SCROLL_DIV;
        // dpat1->raxyz[2] += SCROLL_DIV;

        // if(scanf("%s", input))
        memset(input, 0, sizeof(input));
        if (read(fd, input, sizeof(input)) > 0)
        {
            //x scroll
            if (input[0] == '3')
                dpat1->raxyz[0] += SCROLL_DIV * strlen(input);
            else if (input[0] == '1')
                dpat1->raxyz[0] -= SCROLL_DIV * strlen(input);
            //y scroll
            else if (input[0] == 'w')
                dpat1->raxyz[1] += SCROLL_DIV * strlen(input);
            else if (input[0] == '2')
                dpat1->raxyz[1] -= SCROLL_DIV * strlen(input);
            //z scroll
            else if (input[0] == 'q')
                dpat1->raxyz[2] += SCROLL_DIV * strlen(input);
            else if (input[0] == 'e')
                dpat1->raxyz[2] -= SCROLL_DIV * strlen(input);

            //z move
            if (input[0] == 's')
                dpat1->mvxyz[2] += MOVE_DIV * strlen(input);
            else if (input[0] == 'x')
                dpat1->mvxyz[2] -= MOVE_DIV * strlen(input);
            //y move
            else if (input[0] == 'z')
                dpat1->mvxyz[1] += MOVE_DIV * strlen(input);
            else if (input[0] == 'c')
                dpat1->mvxyz[1] -= MOVE_DIV * strlen(input);
            //x move
            else if (input[0] == 'd')
                dpat1->mvxyz[0] += MOVE_DIV * strlen(input);
            else if (input[0] == 'a')
                dpat1->mvxyz[0] -= MOVE_DIV * strlen(input);

            else if (input[0] == 'r')
            {
                _3D_reset(dpat1);
                // _3D_reset(dpat2);
                posture_reset();
            }
        }

        delayus(INTERVALUS);
    }
}
