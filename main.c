
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pseudo3D.h"
#include "view.h"

#define SCROLL_DIV  (_3D_PI/16)
#define MOVE_DIV  10

int main(void)
{
    char input[16];

    //初始化一个多边形
    _3D_PointArray_Type *dpat0, *dpat1, *dpat2, *dpat3, *dpat4;

    //XYZ
    if((dpat0 = _3D_pointArray_init(6, 
        _3D_XYZ_ScanLen*1.00, 0.00, 0.00, 0x800000, 
        -_3D_XYZ_ScanLen*1.00, 0.00, 0.00, 0x800000, 
        0.00, _3D_XYZ_ScanLen*1.00, 0.00, 0x008080, 
        0.00, -_3D_XYZ_ScanLen*1.00, 0.00, 0x008080, 
        0.00, 0.00, _3D_XYZ_ScanLen*1.00, 0x008000, 
        0.00, 0.00, -_3D_XYZ_ScanLen*1.00, 0x008000
        )) == NULL)
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
    if((dpat1 = _3D_pointArray_init(8, 
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

    //棱形
    if((dpat2 = _3D_pointArray_init(4, 
        0.00, 0.00+100, 50.00, 0xFF00FF, 
        -20.00, -30.00+100, 0.00, 0xFFFF00, 
        -20.00, 30.00+100, 0.00, 0x00FFFF, 
        40.00, 0.00+100, 0.00, 0xFF8000
        )) == NULL)
    {
        printf("_3D_pointArray_init failed\r\n");
        return -1;
    }
    _3D_ppLink_add(dpat2, 0xFF0000, 0, 3, 1, 2, 3);
    _3D_ppLink_add(dpat2, 0x00FF00, 1, 1, 2);
    _3D_ppLink_add(dpat2, 0x0000FF, 2, 1, 3);
    _3D_ppLink_add(dpat2, 0xFFFF00, 3, 1, 1);

    //target point
    if((dpat3 = _3D_pointArray_init(2, 
        80.00, 0.00, 0.00, 0xFFFFFF, 
        0.00, 0.00, 0.00, 0xFFFFFF
        )) == NULL)
    {
        printf("_3D_pointArray_init failed\r\n");
        return -1;
    }
    _3D_ppLink_add(dpat3, 0xFFFFFF, 0, 1, 1);
    _3D_comment_add(dpat3, 80, 0, 0, "target", 0, 0x808080);
    
    //machine
    if((dpat4 = _3D_pointArray_init(24, 
        -4.00*2, 16.00*2, 25.00*2, 0x008080, 
        -4.00*2, 8.00*2, 25.00*2, 0x008080, 
        -4.00*2, 16.00*2, 17.00*2, 0x008080, 
        -4.00*2, 8.00*2, 17.00*2, 0x008080, 
        -4.00*2, -17.00*2, 17.00*2, 0x008080, 
        -4.00*2, 4.00*2, 13.00*2, 0x008080, 
        -4.00*2, -13.00*2, 13.00*2, 0x008080, 
        -4.00*2, 4.00*2, -5.00*2, 0x008080, 
        -4.00*2, -13.00*2, -5.00*2, 0x008080, 
        -4.00*2, 16.00*2, -24.00*2, 0x008080, 
        -4.00*2, 8.00*2, -24.00*2, 0x008080, 
        -4.00*2, -17.00*2, -24.00*2, 0x008080,

        4.00*2, 16.00*2, 25.00*2, 0xD0D000, 
        4.00*2, 8.00*2, 25.00*2, 0xD0D000, 
        4.00*2, 16.00*2, 17.00*2, 0xD0D000, 
        4.00*2, 8.00*2, 17.00*2, 0xD0D000, 
        4.00*2, -17.00*2, 17.00*2, 0xD0D000, 
        4.00*2, 4.00*2, 13.00*2, 0xD0D000, 
        4.00*2, -13.00*2, 13.00*2, 0xD0D000, 
        4.00*2, 4.00*2, -5.00*2, 0xD0D000, 
        4.00*2, -13.00*2, -5.00*2, 0xD0D000, 
        4.00*2, 16.00*2, -24.00*2, 0xD0D000, 
        4.00*2, 8.00*2, -24.00*2, 0xD0D000, 
        4.00*2, -17.00*2, -24.00*2, 0xD0D000
        )) == NULL)
    {
        printf("_3D_pointArray_init failed\r\n");
        return -1;
    }
    _3D_ppLink_add(dpat4, 0x008080, 0, 3, 1, 2, 12);
    _3D_ppLink_add(dpat4, 0x008080, 1, 2, 3, 13);
    _3D_ppLink_add(dpat4, 0x008080, 2, 3, 3, 9, 14);
    _3D_ppLink_add(dpat4, 0x008080, 3, 3, 4, 10, 15);
    _3D_ppLink_add(dpat4, 0x008080, 4, 2, 11, 16);
    _3D_ppLink_add(dpat4, 0x008080, 5, 3, 6, 7, 17);
    _3D_ppLink_add(dpat4, 0x008080, 6, 2, 8, 18);
    _3D_ppLink_add(dpat4, 0x008080, 7, 2, 8, 19);
    _3D_ppLink_add(dpat4, 0x008080, 8, 1, 20);
    _3D_ppLink_add(dpat4, 0x008080, 9, 2, 10, 21);
    _3D_ppLink_add(dpat4, 0x008080, 10, 2, 11, 22);
    _3D_ppLink_add(dpat4, 0x008080, 11, 1, 23);
    _3D_ppLink_add(dpat4, 0xD0D000, 12, 2, 13, 14);
    _3D_ppLink_add(dpat4, 0xD0D000, 13, 1, 15);
    _3D_ppLink_add(dpat4, 0xD0D000, 14, 2, 15, 21);
    _3D_ppLink_add(dpat4, 0xD0D000, 15, 2, 16, 22);
    _3D_ppLink_add(dpat4, 0xD0D000, 16, 1, 23);
    _3D_ppLink_add(dpat4, 0xD0D000, 17, 2, 18, 19);
    _3D_ppLink_add(dpat4, 0xD0D000, 18, 1, 20);
    _3D_ppLink_add(dpat4, 0xD0D000, 19, 1, 20);
    _3D_ppLink_add(dpat4, 0xD0D000, 21, 1, 22);
    _3D_ppLink_add(dpat4, 0xD0D000, 22, 1, 23);

    //初始转角
    // dpat1->raxyz[0] = _3D_PI/8;
    // dpat1->raxyz[1] = _3D_PI/8;
    // dpat1->raxyz[2] = _3D_PI/8;

    while(1)
    {
        //
        PRINT_CLEAR();

        //
        memcpy(dpat2->raxyz, dpat1->raxyz, sizeof(dpat1->raxyz));
        memcpy(dpat3->raxyz, dpat1->raxyz, sizeof(dpat1->raxyz));
        memcpy(dpat4->raxyz, dpat1->raxyz, sizeof(dpat1->raxyz));
        //
        memcpy(dpat2->mvxyz, dpat1->mvxyz, sizeof(dpat1->mvxyz));
        memcpy(dpat3->mvxyz, dpat1->mvxyz, sizeof(dpat1->mvxyz));
        memcpy(dpat4->mvxyz, dpat1->mvxyz, sizeof(dpat1->mvxyz));

        _3D_angle_to_xyz(dpat4);
        _3D_angle_to_xyz(dpat3);
        _3D_angle_to_xyz(dpat2);
        _3D_angle_to_xyz(dpat1);
        
        _3D_draw(VIEW_X_SIZE/2, VIEW_Y_SIZE/2, dpat0);

        // _3D_draw(VIEW_X_SIZE/4, VIEW_Y_SIZE/4, dpat4);
        _3D_draw(VIEW_X_SIZE/2, VIEW_Y_SIZE/2, dpat3);
        _3D_draw(VIEW_X_SIZE/2, VIEW_Y_SIZE/2, dpat2);
        _3D_draw(VIEW_X_SIZE/2, VIEW_Y_SIZE/2, dpat1);
        
        //
        PRINT_EN();

        // dpat1->raxyz[0] += SCROLL_DIV;
        // dpat1->raxyz[1] += SCROLL_DIV;
        // dpat1->raxyz[2] += SCROLL_DIV;

        if(scanf("%s", input))
        {
            //x scroll
            if(input[0] == '3')
                dpat1->raxyz[0] += SCROLL_DIV*strlen(input);
            else if(input[0] == '1')
                dpat1->raxyz[0] -= SCROLL_DIV*strlen(input);
            //y scroll
            else if(input[0] == 'w')
                dpat1->raxyz[1] += SCROLL_DIV*strlen(input);
            else if(input[0] == '2')
                dpat1->raxyz[1] -= SCROLL_DIV*strlen(input);
            //z scroll
            else if(input[0] == 'q')
                dpat1->raxyz[2] += SCROLL_DIV*strlen(input);
            else if(input[0] == 'e')
                dpat1->raxyz[2] -= SCROLL_DIV*strlen(input);
            
            //z move
            if(input[0] == 's')
                dpat1->mvxyz[2] += MOVE_DIV*strlen(input);
            else if(input[0] == 'x')
                dpat1->mvxyz[2] -= MOVE_DIV*strlen(input);
            //y move
            else if(input[0] == 'z')
                dpat1->mvxyz[1] += MOVE_DIV*strlen(input);
            else if(input[0] == 'c')
                dpat1->mvxyz[1] -= MOVE_DIV*strlen(input);
            //x move
            else if(input[0] == 'd')
                dpat1->mvxyz[0] += MOVE_DIV*strlen(input);
            else if(input[0] == 'a')
                dpat1->mvxyz[0] -= MOVE_DIV*strlen(input);

            else if(input[0] == 'r')
            {
                _3D_reset(dpat1);
                _3D_reset(dpat2);
                _3D_reset(dpat3);
                _3D_reset(dpat4);
            }
        }

        usleep(100000);
    }
}


