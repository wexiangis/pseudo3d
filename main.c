
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tft_3d.h"
#include "view.h"

#define SCROLL_DIV  (_3D_PI/16)
#define MOVE_DIV  10

int main(void)
{
    char input[16];

    //初始化一个多边形
    _3D_PointArray_Type *ddat, *ddat2, *ddat3;

    //长方体
    if((ddat = _3D_pointArray_init(8, 
        40.00, 30.00, 50.00, 0xFF00FF, 
        40.00, -30.00, 50.00, 0xFFFF00, 
        -40.00, -30.00, 50.00, 0x00FFFF, 
        -40.00, 30.00, 50.00, 0xFF8000, 
        -40.00, 30.00, -50.00, 0xFF00FF, 
        -40.00, -30.00, -50.00, 0xFFFF00, 
        40.00, -30.00, -50.00, 0x00FFFF, 
        40.00, 30.00, -50.00, 0xFF8000
        )) == NULL)
    {
        printf("_3D_pointArray_init failed\r\n");
        return -1;
    }
    _3D_ppLink_add(ddat, 0, 3, 1, 3, 7);
    _3D_ppLink_add(ddat, 1, 2, 2, 6);
    _3D_ppLink_add(ddat, 2, 2, 3, 5);
    _3D_ppLink_add(ddat, 3, 1, 4);
    _3D_ppLink_add(ddat, 4, 2, 5, 7);
    _3D_ppLink_add(ddat, 5, 1, 6);
    _3D_ppLink_add(ddat, 6, 1, 7);
    _3D_comment_add(ddat, 40.00, 30.00, 50.00, "A", 0xFFFF00);
    _3D_comment_add(ddat, 40.00, -30.00, 50.00, "B", 0x00FF00);
    _3D_comment_add(ddat, -40.00, -30.00, 50.00, "C", 0x8080FF);
    _3D_comment_add(ddat, -40.00, 30.00, 50.00, "D", 0xFF0000);
    _3D_comment_add(ddat, -40.00, 30.00, -50.00, "E", 0xFF00FF);
    _3D_comment_add(ddat, -40.00, -30.00, -50.00, "F", 0x00FFFF);
    _3D_comment_add(ddat, 40.00, -30.00, -50.00, "G", 0xFF8000);
    _3D_comment_add(ddat, 40.00, 30.00, -50.00, "H", 0x0080FF);

    //棱形
    if((ddat2 = _3D_pointArray_init(4, 
        0.00, 0.00+100, 50.00, 0xFF00FF, 
        -20.00, -30.00+100, 0.00, 0xFFFF00, 
        -20.00, 30.00+100, 0.00, 0x00FFFF, 
        40.00, 0.00+100, 0.00, 0xFF8000
        )) == NULL)
    {
        printf("_3D_pointArray_init failed\r\n");
        return -1;
    }
    _3D_ppLink_add(ddat2, 0, 3, 1, 2, 3);
    _3D_ppLink_add(ddat2, 1, 1, 2);
    _3D_ppLink_add(ddat2, 2, 1, 3);
    _3D_ppLink_add(ddat2, 3, 1, 1);

    //XYZ
    if((ddat3 = _3D_pointArray_init(6, 
        _3D_XYZ_ScanLen*1.00, 0.00, 0.00, 0xFF0000, 
        -_3D_XYZ_ScanLen*1.00, 0.00, 0.00, 0xFF0000, 
        0.00, _3D_XYZ_ScanLen*1.00, 0.00, 0x00FFFF, 
        0.00, -_3D_XYZ_ScanLen*1.00, 0.00, 0x00FFFF, 
        0.00, 0.00, _3D_XYZ_ScanLen*1.00, 0x00FF00, 
        0.00, 0.00, -_3D_XYZ_ScanLen*1.00, 0x00FF00
        )) == NULL)
    {
        printf("_3D_pointArray_init failed\r\n");
        return -1;
    }
    _3D_ppLink_add(ddat3, 0, 1, 1);
    _3D_ppLink_add(ddat3, 2, 1, 3);
    _3D_ppLink_add(ddat3, 4, 1, 5);
    _3D_comment_add(ddat3, _3D_XYZ_ScanLen, 0, 0, "X", 0xFF0000);
    _3D_comment_add(ddat3, 0, _3D_XYZ_ScanLen, 0, "Y", 0x00FFFF);
    _3D_comment_add(ddat3, 0, 0, _3D_XYZ_ScanLen, "Z", 0x00FF00);

    //初始转角
    // ddat->raxyz[0] = _3D_PI/8;
    // ddat->raxyz[1] = _3D_PI/8;
    // ddat->raxyz[2] = _3D_PI/8;

    while(1)
    {
        //
        PRINT_CLEAR();

        //
        memcpy(ddat2->raxyz, ddat->raxyz, sizeof(ddat->raxyz));
        memcpy(ddat3->raxyz, ddat->raxyz, sizeof(ddat->raxyz));

        _3D_angle_to_xyz(ddat3);
        _3D_angle_to_xyz(ddat2);
        _3D_angle_to_xyz(ddat);
        
        _3D_draw(VIEW_X_SIZE/2, VIEW_Y_SIZE/2, ddat3);
        _3D_draw(VIEW_X_SIZE/2, VIEW_Y_SIZE/2, ddat2);
        _3D_draw(VIEW_X_SIZE/2, VIEW_Y_SIZE/2, ddat);
        
        //
        PRINT_EN();

        // ddat->raxyz[0] += SCROLL_DIV;
        // ddat->raxyz[1] += SCROLL_DIV;
        // ddat->raxyz[2] += SCROLL_DIV;

        if(scanf("%s", input))
        {
            //x scroll
            if(input[0] == '3')
                ddat->raxyz[0] += SCROLL_DIV*strlen(input);
            else if(input[0] == '1')
                ddat->raxyz[0] -= SCROLL_DIV*strlen(input);
            //y scroll
            else if(input[0] == 'w')
                ddat->raxyz[1] += SCROLL_DIV*strlen(input);
            else if(input[0] == '2')
                ddat->raxyz[1] -= SCROLL_DIV*strlen(input);
            //z scroll
            else if(input[0] == 'q')
                ddat->raxyz[2] += SCROLL_DIV*strlen(input);
            else if(input[0] == 'e')
                ddat->raxyz[2] -= SCROLL_DIV*strlen(input);
            
            //z move
            if(input[0] == 's')
                ddat->mvxyz[2] += MOVE_DIV*strlen(input);
            else if(input[0] == 'x')
                ddat->mvxyz[2] -= MOVE_DIV*strlen(input);
            //y move
            else if(input[0] == 'z')
                ddat->mvxyz[1] += MOVE_DIV*strlen(input);
            else if(input[0] == 'c')
                ddat->mvxyz[1] -= MOVE_DIV*strlen(input);
            //x move
            else if(input[0] == 'd')
                ddat->mvxyz[0] += MOVE_DIV*strlen(input);
            else if(input[0] == 'a')
                ddat->mvxyz[0] -= MOVE_DIV*strlen(input);

            else if(input[0] == 'r')
            {
                _3D_reset(ddat);
                _3D_reset(ddat2);
                _3D_reset(ddat3);
            }
        }

        usleep(100000);
    }
}


