
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
#include "pe_math.h"
#include "fbmap.h"

//使用陀螺仪模块
#define ENABLE_MPU6050
#ifdef ENABLE_MPU6050
#include "posture.h"
#include "fbmap.h"
#include "dot.h"
#include "wave.h"
#define MPU6050_INTERVALMS 10 //sample freq ms
#endif

//采样间隔
#define INTERVALMS 10 //screen freq ms
//旋转分度值
#define DIV_SCROLL (P3D_PI / 16)
//平移分度值
#define DIV_MOVE 10

int main(int argc, char **argv)
{
    //初始化一个多边形
    P3D_PointArray_Type *dpat0, *dpat1, *dpat2, *dpat3;
    //测试点
    float xyz[3];
    float raxyz[3] = {0};
    float mvxyz[3] = {0};
    float quat[4] = {1, 0, 0, 0};
    //终端输入
    char input[16];
    DELAY_US_INIT;
#ifdef ENABLE_MPU6050
    int log_count = 0;
    int fd;
    //姿态结构体
    PostureStruct *ps;
    //示波器2个
    Wave_Struct *ws1, *ws2;
    //打点器1个
    Dot_Struct *ds;

    // open console
    if (argc > 1)
        fd = open(argv[1], O_RDONLY);
    else
        fd = open("/dev/console", O_RDONLY);
    //非阻塞设置
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
#endif

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
    dpat1->_matrix_mode = 1; //使用zyx旋转矩阵

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
    dpat2->_matrix_mode = 1; //使用zyx旋转矩阵

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
    dpat3->_matrix_mode = 1; //使用zyx旋转矩阵

#ifdef ENABLE_MPU6050
    //初始化姿态计算器
    ps = pe_init(MPU6050_INTERVALMS, NULL, NULL);
    fb_output(NULL, 0, 0, 0, 0);
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
        DELAY_US(INTERVALMS * 1000);

#ifdef ENABLE_MPU6050

#if 0
        wave_load(ws1, 0, (short)(ps->gyrXYZ[0] * 50));
        wave_load(ws1, 1, (short)(ps->gyrXYZ[1] * 50));
        wave_load(ws1, 2, (short)(ps->gyrXYZ[2] * 50));

        wave_load(ws2, 0, (short)(ps->accXYZ[0] * 10000));
        wave_load(ws2, 1, (short)(ps->accXYZ[1] * 10000));
        wave_load(ws2, 2, (short)(ps->accXYZ[2] * 10000));
#elif 1
        wave_load(ws1, 0, 10000);//基准线
        wave_load(ws1, 1, (short)(ps->speX * 10000) + 10000);
        wave_load(ws1, 3, (short)(ps->speY * 10000) + 10000);

        wave_load(ws1, 4, (short)(ps->speZ * 10000));

        wave_load(ws1, 5, (short)(ps->speXYZ * 10000) - 10000);
        wave_load(ws1, 6, -10000);//基准线

        wave_load(ws2, 0, 10000);//基准线
        wave_load(ws2, 1, (short)(ps->gX * 50000) + 10000);
        wave_load(ws2, 3, (short)(ps->gY * 50000) + 10000);

        wave_load(ws2, 4, (short)((ps->gZ - 1.0) * 50000));

        wave_load(ws2, 5, (short)((ps->gXYZ - 1.0) * 50000) - 10000);
        wave_load(ws2, 6, -10000);//基准线
#endif

        dot_set(ds, ps->gX, 0, 0xFF0000);
        dot_set(ds, 0, ps->gY, 0x0000FF);
        dot_set(ds, ps->gX, ps->gY, 0x00FF00);

        wave_output(ws1);
        wave_output(ws2);
        dot_output(ds);

        //拷贝姿态角度+调整
        memcpy(dpat1->raxyz, ps->rollXYZ, sizeof(float) * 3);
        dpat1->raxyz[0] = -dpat1->raxyz[0];
        dpat1->raxyz[2] = -dpat1->raxyz[2];
        memcpy(dpat2->raxyz, ps->accRollXYZ, sizeof(float) * 3);
        dpat2->raxyz[0] = -dpat2->raxyz[0];
        dpat2->raxyz[2] = -dpat2->raxyz[2];
        memcpy(dpat3->raxyz, ps->gyrRollXYZ, sizeof(float) * 3);
        dpat3->raxyz[0] = -dpat3->raxyz[0];
        dpat3->raxyz[2] = -dpat3->raxyz[2];

        log_count += INTERVALMS;
        if (log_count >= 20)
        {
            log_count = 0;

            // printf(" acc %8.4f %8.4f %8.4f gyr %8.4f %8.4f %8.4f roll %8.4f %8.4f %8.4f gyrRoll %8.4f %8.4f %8.4f \r\n", 
            //     ps->accXYZ[0], ps->accXYZ[1], ps->accXYZ[2],
            //     ps->gyrXYZ[0], ps->gyrXYZ[1], ps->gyrXYZ[2], 
            //     ps->rollXYZ[0], ps->rollXYZ[1], ps->rollXYZ[2],
            //     ps->gyrRollXYZ[0], ps->gyrRollXYZ[1], ps->gyrRollXYZ[2]);

            // printf(" spe %8.4f %8.4f %8.4f mov %8.4f %8.4f %8.4f \r\n",
            //     ps->speX, ps->speY, ps->speZ,
            //     ps->movX, ps->movY, ps->movZ);

            // printf(" g %8.4f %8.4f %8.4f gXYZ %8.4f \r\n",
            //     ps->gX, ps->gY, ps->gZ, ps->gXYZ);
        }
        //逆矩阵测试,查看重力加速的合向量在空间坐标系中的位置
        xyz[0] = -ps->accXYZ[0] * 100;
        xyz[1] = ps->accXYZ[1] * 100;
        xyz[2] = -ps->accXYZ[2] * 100;
        matrix_zyx(dpat1->raxyz, xyz, xyz);
#endif

        PRINT_CLEAR();

        p3d_draw(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 2, dpat0);
        p3d_draw(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 4, dpat1);
        p3d_draw(VIEW_X_SIZE / 4, VIEW_Y_SIZE / 4 * 3, dpat2);
        p3d_draw(VIEW_X_SIZE / 4 * 3, VIEW_Y_SIZE / 4 * 3, dpat3);
        p3d_draw2(VIEW_X_SIZE / 2, VIEW_Y_SIZE / 2, 0xFF8000, xyz);

        PRINT_EN();

#ifdef ENABLE_MPU6050
        memset(input, 0, sizeof(input));
        if (read(fd, input, sizeof(input)) > 0)
#else
        if(scanf("%s", input))
#endif
        {
            //x scroll
            if (input[0] == '3')
                raxyz[0] += DIV_SCROLL * strlen(input);
            else if (input[0] == '1')
                raxyz[0] -= DIV_SCROLL * strlen(input);
            //y scroll
            else if (input[0] == 'w')
                raxyz[1] += DIV_SCROLL * strlen(input);
            else if (input[0] == '2')
                raxyz[1] -= DIV_SCROLL * strlen(input);
            //z scroll
            else if (input[0] == 'q')
                raxyz[2] += DIV_SCROLL * strlen(input);
            else if (input[0] == 'e')
                raxyz[2] -= DIV_SCROLL * strlen(input);

            //z move
            if (input[0] == 's')
                mvxyz[2] += DIV_MOVE * strlen(input);
            else if (input[0] == 'x')
                mvxyz[2] -= DIV_MOVE * strlen(input);
            //y move
            else if (input[0] == 'z')
                mvxyz[1] += DIV_MOVE * strlen(input);
            else if (input[0] == 'c')
                mvxyz[1] -= DIV_MOVE * strlen(input);
            //x move
            else if (input[0] == 'd')
                mvxyz[0] += DIV_MOVE * strlen(input);
            else if (input[0] == 'a')
                mvxyz[0] -= DIV_MOVE * strlen(input);

            //reset
            else if (input[0] == 'r')
            {
                memset(raxyz, 0, sizeof(raxyz));
                memset(mvxyz, 0, sizeof(mvxyz));
                memset(quat, 0, sizeof(quat));
                quat[0] = 1;
                p3d_reset(dpat1);
                p3d_reset(dpat2);
                p3d_reset(dpat3);
#ifdef ENABLE_MPU6050
                pe_reset(ps);
                dot_clear(ds);
#endif
            }

            quat_diff(quat, raxyz);
            quat_to_pry(quat, raxyz);

            memcpy(dpat1->raxyz, raxyz, sizeof(float) * 3);
            memcpy(dpat2->raxyz, raxyz, sizeof(float) * 3);
            memcpy(dpat3->raxyz, raxyz, sizeof(float) * 3);

            memcpy(dpat1->mvxyz, mvxyz, sizeof(float) * 3);
            memcpy(dpat2->mvxyz, mvxyz, sizeof(float) * 3);
            memcpy(dpat3->mvxyz, mvxyz, sizeof(float) * 3);

            memset(raxyz, 0, sizeof(float) * 3);

            printf("rX/%.4f, rY/%.4f, rZ/%.4f \r\n", 
                raxyz[0], raxyz[1], raxyz[2]);
        }
    }
}
