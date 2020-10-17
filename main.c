
/*
 *  通过mpu6050计算姿态信息，并输出成3D图像显示
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "delayus.h"
#include "pseudo3d.h"
#include "view.h"

//使用陀螺仪模块
#define ENABLE_MPU6050 0
#if (ENABLE_MPU6050)
#include "posture.h"
#include "wave.h"
#endif

//采样间隔
#define INTERVALUS 50000
//旋转分度值
#define DIV_SCROLL (P3D_PI / 16)
//平移分度值
#define DIV_MOVE 10

int main(int argc, char **argv)
{
    //初始化一个多边形
    P3D_PointArray_Type *dpat0, *dpat1, *dpat2, *dpat3;

    char input[16];
    int fd;

    // open console
    if (argc > 1)
        fd = open(argv[1], O_RDONLY);
    else
        fd = open("/dev/console", O_RDONLY);
    //非阻塞设置
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    //XYZ
    if ((dpat0 = p3d_pointArray_init(6,
                                     P3D_XYZ_LEN * 1.00, 0.00, 0.00, 0x800000,
                                     -P3D_XYZ_LEN * 1.00, 0.00, 0.00, 0x800000,
                                     0.00, P3D_XYZ_LEN * 1.00, 0.00, 0x008080,
                                     0.00, -P3D_XYZ_LEN * 1.00, 0.00, 0x008080,
                                     0.00, 0.00, P3D_XYZ_LEN * 1.00, 0x008000,
                                     0.00, 0.00, -P3D_XYZ_LEN * 1.00, 0x008000)) == NULL)
    {
        printf("p3d_pointArray_init failed\r\n");
        return -1;
    }
    p3d_ppLink_add(dpat0, 0x800000, 0, 1, 1);
    p3d_ppLink_add(dpat0, 0x008000, 2, 1, 3);
    p3d_ppLink_add(dpat0, 0x008080, 4, 1, 5);
    p3d_comment_add(dpat0, P3D_XYZ_LEN, 0, 0, "X", 0, 0x800000);
    p3d_comment_add(dpat0, 0, P3D_XYZ_LEN, 0, "Y", 0, 0x008080);
    p3d_comment_add(dpat0, 0, 0, P3D_XYZ_LEN, "Z", 0, 0x008000);

    //长方体
    if ((dpat1 = p3d_pointArray_init(8,
                                     40.00, 30.00, 50.00, 0xFF00FF,
                                     40.00, -30.00, 50.00, 0xFFFF00,
                                     -40.00, -30.00, 50.00, 0x00FFFF,
                                     -40.00, 30.00, 50.00, 0xFF8000,
                                     -40.00, 30.00, -50.00, 0xFF00FF,
                                     -40.00, -30.00, -50.00, 0xFFFF00,
                                     40.00, -30.00, -50.00, 0x00FFFF,
                                     40.00, 30.00, -50.00, 0xFF8000)) == NULL)
    {
        printf("p3d_pointArray_init failed\r\n");
        return -1;
    }
    p3d_ppLink_add(dpat1, 0xFF0000, 0, 3, 1, 3, 7);
    p3d_ppLink_add(dpat1, 0x00FF00, 1, 2, 2, 6);
    p3d_ppLink_add(dpat1, 0x0000FF, 2, 2, 3, 5);
    p3d_ppLink_add(dpat1, 0xFFFF00, 3, 1, 4);
    p3d_ppLink_add(dpat1, 0xFF00FF, 4, 2, 5, 7);
    p3d_ppLink_add(dpat1, 0x00FFFF, 5, 1, 6);
    p3d_ppLink_add(dpat1, 0xFF8000, 6, 1, 7);
    p3d_comment_add(dpat1, 40.00, 30.00, 50.00, "A", 0, 0xFFFF00);
    p3d_comment_add(dpat1, 40.00, -30.00, 50.00, "B", 0, 0x00FF00);
    p3d_comment_add(dpat1, -40.00, -30.00, 50.00, "C", 0, 0x8080FF);
    p3d_comment_add(dpat1, -40.00, 30.00, 50.00, "D", 0, 0xFF0000);
    p3d_comment_add(dpat1, -40.00, 30.00, -50.00, "E", 0, 0xFF00FF);
    p3d_comment_add(dpat1, -40.00, -30.00, -50.00, "F", 0, 0x00FFFF);
    p3d_comment_add(dpat1, 40.00, -30.00, -50.00, "G", 0, 0xFF8000);
    p3d_comment_add(dpat1, 40.00, 30.00, -50.00, "H", 0, 0x0080FF);
    dpat1->_matrix_mode = 0;//使用左乘

    //长方体2
    if ((dpat2 = p3d_pointArray_init(8,
                                     40.00, 30.00, 50.00, 0xFF00FF,
                                     40.00, -30.00, 50.00, 0xFFFF00,
                                     -40.00, -30.00, 50.00, 0x00FFFF,
                                     -40.00, 30.00, 50.00, 0xFF8000,
                                     -40.00, 30.00, -50.00, 0xFF00FF,
                                     -40.00, -30.00, -50.00, 0xFFFF00,
                                     40.00, -30.00, -50.00, 0x00FFFF,
                                     40.00, 30.00, -50.00, 0xFF8000)) == NULL)
    {
        printf("p3d_pointArray_init failed\r\n");
        return -1;
    }
    p3d_ppLink_add(dpat2, 0xFF0000, 0, 3, 1, 3, 7);
    p3d_ppLink_add(dpat2, 0x00FF00, 1, 2, 2, 6);
    p3d_ppLink_add(dpat2, 0x0000FF, 2, 2, 3, 5);
    p3d_ppLink_add(dpat2, 0xFFFF00, 3, 1, 4);
    p3d_ppLink_add(dpat2, 0xFF00FF, 4, 2, 5, 7);
    p3d_ppLink_add(dpat2, 0x00FFFF, 5, 1, 6);
    p3d_ppLink_add(dpat2, 0xFF8000, 6, 1, 7);
    p3d_comment_add(dpat2, 40.00, 30.00, 50.00, "A", 0, 0xFFFF00);
    p3d_comment_add(dpat2, 40.00, -30.00, 50.00, "B", 0, 0x00FF00);
    p3d_comment_add(dpat2, -40.00, -30.00, 50.00, "C", 0, 0x8080FF);
    p3d_comment_add(dpat2, -40.00, 30.00, 50.00, "D", 0, 0xFF0000);
    p3d_comment_add(dpat2, -40.00, 30.00, -50.00, "E", 0, 0xFF00FF);
    p3d_comment_add(dpat2, -40.00, -30.00, -50.00, "F", 0, 0x00FFFF);
    p3d_comment_add(dpat2, 40.00, -30.00, -50.00, "G", 0, 0xFF8000);
    p3d_comment_add(dpat2, 40.00, 30.00, -50.00, "H", 0, 0x0080FF);
    dpat2->_matrix_mode = 1;//使用右乘

    //长方体3
    if ((dpat3 = p3d_pointArray_init(8,
                                     40.00, 30.00, 50.00, 0xFF00FF,
                                     40.00, -30.00, 50.00, 0xFFFF00,
                                     -40.00, -30.00, 50.00, 0x00FFFF,
                                     -40.00, 30.00, 50.00, 0xFF8000,
                                     -40.00, 30.00, -50.00, 0xFF00FF,
                                     -40.00, -30.00, -50.00, 0xFFFF00,
                                     40.00, -30.00, -50.00, 0x00FFFF,
                                     40.00, 30.00, -50.00, 0xFF8000)) == NULL)
    {
        printf("p3d_pointArray_init failed\r\n");
        return -1;
    }
    p3d_ppLink_add(dpat3, 0xFF0000, 0, 3, 1, 3, 7);
    p3d_ppLink_add(dpat3, 0x00FF00, 1, 2, 2, 6);
    p3d_ppLink_add(dpat3, 0x0000FF, 2, 2, 3, 5);
    p3d_ppLink_add(dpat3, 0xFFFF00, 3, 1, 4);
    p3d_ppLink_add(dpat3, 0xFF00FF, 4, 2, 5, 7);
    p3d_ppLink_add(dpat3, 0x00FFFF, 5, 1, 6);
    p3d_ppLink_add(dpat3, 0xFF8000, 6, 1, 7);
    p3d_comment_add(dpat3, 40.00, 30.00, 50.00, "A", 0, 0xFFFF00);
    p3d_comment_add(dpat3, 40.00, -30.00, 50.00, "B", 0, 0x00FF00);
    p3d_comment_add(dpat3, -40.00, -30.00, 50.00, "C", 0, 0x8080FF);
    p3d_comment_add(dpat3, -40.00, 30.00, 50.00, "D", 0, 0xFF0000);
    p3d_comment_add(dpat3, -40.00, 30.00, -50.00, "E", 0, 0xFF00FF);
    p3d_comment_add(dpat3, -40.00, -30.00, -50.00, "F", 0, 0x00FFFF);
    p3d_comment_add(dpat3, 40.00, -30.00, -50.00, "G", 0, 0xFF8000);
    p3d_comment_add(dpat3, 40.00, 30.00, -50.00, "H", 0, 0x0080FF);
    dpat3->_matrix_mode = 1;//使用右乘

#if (ENABLE_MPU6050)
    //初始化姿态计算器
    posture_init(INTERVALUS / 1000);
#endif

    while (1)
    {

#if (ENABLE_MPU6050)

        wave_load(0, (short)(posture_getACX() * 10000));
        wave_load(1, (short)(posture_getACY() * 10000));
        wave_load(2, (short)(posture_getACZ() * 10000));
        wave_load(3, (short)(posture_getAGX() * 10000));
        wave_load(4, (short)(posture_getAGY() * 10000));
        wave_load(5, (short)(posture_getAGZ() * 10000));

        wave_refresh();

        dpat1->raxyz[0] = posture_getACX();
        dpat1->raxyz[1] = posture_getACY();
        dpat1->raxyz[2] = posture_getACZ();
        dpat2->raxyz[0] = posture_getAGX();
        dpat2->raxyz[1] = posture_getAGY();
        dpat2->raxyz[2] = posture_getAGZ();
        dpat3->raxyz[0] = posture_getX();
        dpat3->raxyz[1] = posture_getY();
        dpat3->raxyz[2] = posture_getZ();

        printf("x/%.4f y/%.4f z/%.4f -- x/%.4f y/%.4f z/%.4f -- x/%.4f y/%.4f z/%.4f"
               " -- x/%04d y/%04d z/%04d -- x/%04d y/%04d z/%04d\r\n",
               posture_getACX(), posture_getACY(), posture_getACZ(),
               posture_getAGX(), posture_getAGY(), posture_getAGZ(),
               posture_getX(), posture_getY(), posture_getZ(),
               posture_getAGXVal(), posture_getAGYVal(), posture_getAGZVal(),
               posture_getACXVal(), posture_getACYVal(), posture_getACZVal());

#endif

        PRINT_CLEAR();

        p3d_angle_to_xyz(dpat1);
        p3d_angle_to_xyz(dpat2);
        p3d_angle_to_xyz(dpat3);

        p3d_draw(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 2, dpat0);
        p3d_draw(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 4, dpat1);
        p3d_draw(VIEW_X_SIZE / 4, VIEW_Y_SIZE / 4 * 3, dpat2);
        p3d_draw(VIEW_X_SIZE / 4 * 3, VIEW_Y_SIZE / 4 * 3, dpat3);

        PRINT_EN();

        // dpat1->raxyz[0] += DIV_SCROLL;
        // dpat1->raxyz[1] += DIV_SCROLL;
        // dpat1->raxyz[2] += DIV_SCROLL;

        // if(scanf("%s", input))
        memset(input, 0, sizeof(input));
        if (read(fd, input, sizeof(input)) > 0)
        {
            //x scroll
            if (input[0] == '3')
                dpat1->raxyz[0] += DIV_SCROLL * strlen(input);
            else if (input[0] == '1')
                dpat1->raxyz[0] -= DIV_SCROLL * strlen(input);
            //y scroll
            else if (input[0] == 'w')
                dpat1->raxyz[1] += DIV_SCROLL * strlen(input);
            else if (input[0] == '2')
                dpat1->raxyz[1] -= DIV_SCROLL * strlen(input);
            //z scroll
            else if (input[0] == 'q')
                dpat1->raxyz[2] += DIV_SCROLL * strlen(input);
            else if (input[0] == 'e')
                dpat1->raxyz[2] -= DIV_SCROLL * strlen(input);

            //z move
            if (input[0] == 's')
                dpat1->mvxyz[2] += DIV_MOVE * strlen(input);
            else if (input[0] == 'x')
                dpat1->mvxyz[2] -= DIV_MOVE * strlen(input);
            //y move
            else if (input[0] == 'z')
                dpat1->mvxyz[1] += DIV_MOVE * strlen(input);
            else if (input[0] == 'c')
                dpat1->mvxyz[1] -= DIV_MOVE * strlen(input);
            //x move
            else if (input[0] == 'd')
                dpat1->mvxyz[0] += DIV_MOVE * strlen(input);
            else if (input[0] == 'a')
                dpat1->mvxyz[0] -= DIV_MOVE * strlen(input);

            //reset
            else if (input[0] == 'r')
            {
                p3d_reset(dpat1);
                p3d_reset(dpat2);
                p3d_reset(dpat3);
#if (ENABLE_MPU6050)
                posture_reset();
#endif
            }

            memcpy(dpat2->raxyz, dpat1->raxyz, sizeof(dpat1->raxyz));
            memcpy(dpat3->raxyz, dpat1->raxyz, sizeof(dpat1->raxyz));

            memcpy(dpat2->mvxyz, dpat1->mvxyz, sizeof(dpat1->mvxyz));
            memcpy(dpat3->mvxyz, dpat1->mvxyz, sizeof(dpat1->mvxyz));

            printf("rX/%.4f, rY/%.4f, rZ/%.4f \r\n", dpat1->raxyz[0], dpat1->raxyz[1], dpat1->raxyz[2]);
        }

        delayus(INTERVALUS);
    }
}
