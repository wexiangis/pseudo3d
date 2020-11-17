
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
#define ENABLE_MPU6050 1
#if (ENABLE_MPU6050)
#include "posture.h"
#include "fbmap.h"
#include "dot.h"
#include "wave.h"
#define MPU6050_INTERVALMS 10 //sample freq ms
#endif

//采样间隔
#define INTERVALUS 10000 //screen freq us
//旋转分度值
#define DIV_SCROLL (P3D_PI / 16)
//平移分度值
#define DIV_MOVE 10

int main(int argc, char **argv)
{
    //初始化一个多边形
    P3D_PointArray_Type *dpat0, *dpat1, *dpat2, *dpat3;
    //终端输入
    char input[16];
    int fd;
    //测试点
    double xyz[3];
#if (ENABLE_MPU6050)
    //姿态结构体
    PostureStruct *ps;
    //示波器2个
    Wave_Struct *ws1, *ws2;
    //打点器1个
    Dot_Struct *ds;
#endif
    int log_count = 0;
    DELAY_US_INIT;

    // open console
    if (argc > 1)
        fd = open(argv[1], O_RDONLY);
    else
        fd = open("/dev/console", O_RDONLY);
    //非阻塞设置
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    //XYZ
    if ((dpat0 = p3d_init(6,
        P3D_XYZ_LEN * 1.00, 0.00, 0.00, 0x800000,
        -P3D_XYZ_LEN * 1.00, 0.00, 0.00, 0x800000,
        0.00, P3D_XYZ_LEN * 1.00, 0.00, 0x008080,
        0.00, -P3D_XYZ_LEN * 1.00, 0.00, 0x008080,
        0.00, 0.00, P3D_XYZ_LEN * 1.00, 0x008000,
        0.00, 0.00, -P3D_XYZ_LEN * 1.00, 0x008000)) == NULL)
    {
        printf("p3d_init failed\r\n");
        return -1;
    }
    p3d_ppLink_add(dpat0, 0x800000, 0, 1, 1);
    p3d_ppLink_add(dpat0, 0x008000, 2, 1, 3);
    p3d_ppLink_add(dpat0, 0x008080, 4, 1, 5);
    p3d_comment_add(dpat0, P3D_XYZ_LEN, 0, 0, "X", 0, 0x800000);
    p3d_comment_add(dpat0, 0, P3D_XYZ_LEN, 0, "Y", 0, 0x008080);
    p3d_comment_add(dpat0, 0, 0, P3D_XYZ_LEN, "Z", 0, 0x008000);

    //长方体
    if ((dpat1 = p3d_init(8,
        40.00, 30.00, 50.00, 0xFF00FF,
        40.00, -30.00, 50.00, 0xFFFF00,
        -40.00, -30.00, 50.00, 0x00FFFF,
        -40.00, 30.00, 50.00, 0xFF8000,
        -40.00, 30.00, -50.00, 0xFF00FF,
        -40.00, -30.00, -50.00, 0xFFFF00,
        40.00, -30.00, -50.00, 0x00FFFF,
        40.00, 30.00, -50.00, 0xFF8000)) == NULL)
    {
        printf("p3d_init failed\r\n");
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
    dpat1->_matrix_mode = 1;//使用zyx旋转矩阵

    //长方体2
    if ((dpat2 = p3d_init(8,
        40.00, 30.00, 50.00, 0xFF00FF,
        40.00, -30.00, 50.00, 0xFFFF00,
        -40.00, -30.00, 50.00, 0x00FFFF,
        -40.00, 30.00, 50.00, 0xFF8000,
        -40.00, 30.00, -50.00, 0xFF00FF,
        -40.00, -30.00, -50.00, 0xFFFF00,
        40.00, -30.00, -50.00, 0x00FFFF,
        40.00, 30.00, -50.00, 0xFF8000)) == NULL)
    {
        printf("p3d_init failed\r\n");
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
    dpat2->_matrix_mode = 1;//使用zyx旋转矩阵

    //长方体3
    if ((dpat3 = p3d_init(8,
        40.00, 30.00, 50.00, 0xFF00FF,
        40.00, -30.00, 50.00, 0xFFFF00,
        -40.00, -30.00, 50.00, 0x00FFFF,
        -40.00, 30.00, 50.00, 0xFF8000,
        -40.00, 30.00, -50.00, 0xFF00FF,
        -40.00, -30.00, -50.00, 0xFFFF00,
        40.00, -30.00, -50.00, 0x00FFFF,
        40.00, 30.00, -50.00, 0xFF8000)) == NULL)
    {
        printf("p3d_init failed\r\n");
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
    dpat3->_matrix_mode = 1;//使用zyx旋转矩阵

#if (ENABLE_MPU6050)
    //初始化姿态计算器
    ps = pe_init(MPU6050_INTERVALMS);
    //示波器初始化(上、下半屏)
    ws1 = wave_init(0, 0, fb_width - VIEW_X_SIZE, fb_height / 2);
    ws2 = wave_init(0, fb_height / 2, fb_width - VIEW_X_SIZE, fb_height / 2);
    //打点器初始化(在姿态图像下方)
    ds = dot_init(
        fb_width - VIEW_X_SIZE, VIEW_Y_SIZE,
        VIEW_X_SIZE, fb_height - VIEW_Y_SIZE,
        -1.5, 1.5, -1.5, 1.5);
#endif

    while (1)
    {
        DELAY_US(INTERVALUS);

#if (ENABLE_MPU6050)

        wave_load(ws1, 0, (short)(ps->rX * 10000));
        wave_load(ws1, 1, (short)(ps->rY * 10000));
        wave_load(ws1, 2, (short)(ps->rZ * 10000));
        //wave_load(ws1, 3, (short)(ps->rAX * 10000));
        //wave_load(ws1, 4, (short)(ps->rAY * 10000));
        //wave_load(ws1, 5, (short)(ps->rAZ * 10000));

        wave_load(ws2, 0, 10000);
        wave_load(ws2, 1, (short)(ps->speX * 10000) + 10000);
        wave_load(ws2, 2, (short)(ps->speY * 10000) + 10000);
        wave_load(ws2, 3, -10000);
        wave_load(ws2, 4, (short)(ps->gX * 50000) - 10000);
        wave_load(ws2, 5, (short)(ps->gY * 50000) - 10000);

        dot_set(ds, ps->gX, 0, 0xFF0000);
        dot_set(ds, 0, ps->gY, 0x0000FF);
        dot_set(ds, ps->gX, ps->gY, 0x00FF00);

        wave_output(ws1);
        wave_output(ws2);
        dot_output(ds);

        dpat1->raxyz[0] = ps->rX;
        dpat1->raxyz[1] = ps->rY;
        dpat1->raxyz[2] = ps->rZ;
        dpat2->raxyz[0] = ps->rAX;
        dpat2->raxyz[1] = ps->rAY;
        dpat2->raxyz[2] = ps->rAZ;
        dpat3->raxyz[0] = ps->rGX;
        dpat3->raxyz[1] = ps->rGY;
        dpat3->raxyz[2] = ps->rGZ;

        log_count += INTERVALUS;
        if (log_count >= 20000) {
        log_count = 0;
#if 0
        printf("x/%.4f y/%.4f z/%.4f AC x/%.4f y/%.4f z/%.4f AG x/%.4f y/%.4f z/%.4f \r\n",
            ps->rX, ps->rY, ps->rZ,
            ps->rAX, ps->rAY, ps->rAZ,
            ps->rGX, ps->rGY, ps->rGZ);
#elif 0
        printf("dir x/%04d y%04d z%04d -- tmp %ld \r\n",
            ps->vCX, ps->vCY, ps->vCZ, ps->temper);
#elif 0
        printf("g %7.4f x/%7.4f y/%7.4f z/%7.4f -- spe x/%7.4f y/%7.4f z/%7.4f -- mov x/%7.4f y/%7.4f z/%7.4f \r\n",
            ps->gXYZ, ps->gX, ps->gY, ps->gZ,
            ps->speX, ps->speY, ps->speZ,
            ps->movX, ps->movY, ps->movZ);
#endif
        }
        //逆矩阵测试,查看重力加速的合向量在空间坐标系中的位置
        xyz[0] = -ps->vAX2 * 100;
        xyz[1] = -ps->vAY2 * 100;
        xyz[2] = -ps->vAZ2 * 100;
        p3d_matrix_zyx(dpat1->raxyz, xyz);
#else
        //逆矩阵测试,该坐标转为物体坐标系后再转回来需没有变化
        xyz[0] = 0;
        xyz[1] = 0;
        xyz[2] = -100;
        //转为物体坐标系
        p3d_matrix_zyx(dpat1->raxyz, xyz);
        //转回空间坐标系
        p3d_matrix_xyz(dpat1->raxyz, xyz);
#endif

        PRINT_CLEAR();

        p3d_draw(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 2, dpat0);
        p3d_draw(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 4, dpat1);
        p3d_draw(VIEW_X_SIZE / 4, VIEW_Y_SIZE / 4 * 3, dpat2);
        p3d_draw(VIEW_X_SIZE / 4 * 3, VIEW_Y_SIZE / 4 * 3, dpat3);
        p3d_draw2(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 2, 0xFF8000, xyz);

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
                pe_reset(ps);
                dot_clear(ds);
#endif
            }

            memcpy(dpat2->raxyz, dpat1->raxyz, sizeof(dpat1->raxyz));
            memcpy(dpat3->raxyz, dpat1->raxyz, sizeof(dpat1->raxyz));

            memcpy(dpat2->mvxyz, dpat1->mvxyz, sizeof(dpat1->mvxyz));
            memcpy(dpat3->mvxyz, dpat1->mvxyz, sizeof(dpat1->mvxyz));

            printf("rX/%.4f, rY/%.4f, rZ/%.4f \r\n", dpat1->raxyz[0], dpat1->raxyz[1], dpat1->raxyz[2]);
        }

    }
}
