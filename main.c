
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tft_3d.h"
#include "view.h"

int main(void)
{
    char input[16];

    //初始化一个多边形
    _3D_PointArray_Type *ddat;

    //三棱锥
    // if((ddat = _3D_pointArray_init(4, 
    //     0.00, 80.00, 0.00, 0xFF00FF, 
    //     0.00, 0.00, 00.00, 0xFFFF00, 
    //     0.00, 0.00, -80.00, 0x00FFFF, 
    //     80.00, 0.00, 0.00, 0xFF8000
    //     )) == NULL)
    // {
    //     printf("_3D_pointArray_init failed\r\n");
    //     return -1;
    // }
    // _3D_ppLink_add(ddat, 0, 2, 2, 3);
    // _3D_ppLink_add(ddat, 1, 1, 3);

    //棱形
    // if((ddat = _3D_pointArray_init(6, 
    //     0.00, 80.00, 0.00, 0xFF00FF, 
    //     80.00, 0.00, 0.00, 0xFFFF00, 
    //     0.00, 0.00, 80.00, 0x00FFFF, 
    //     -80.00, 0.00, 0.00, 0xFF8000, 
    //     0.00, 0.00, -80.00, 0x0080FF, 
    //     0.00, -80.00, 0.00, 0x8000FF
    //     )) == NULL)
    // {
    //     printf("_3D_pointArray_init failed\r\n");
    //     return -1;
    // }
    // _3D_ppLink_add(ddat, 0, 3, 2, 3, 4);
    // _3D_ppLink_add(ddat, 4, 1, 1);
    // _3D_ppLink_add(ddat, 5, 3, 1, 2, 3);

    //长方体 x2
    if((ddat = _3D_pointArray_init(8+8, 
        30.00, 40.00, 50.00, 0xFF00FF, 
        30.00, -40.00, 50.00, 0xFFFF00, 
        -30.00, -40.00, 50.00, 0x00FFFF, 
        -30.00, 40.00, 50.00, 0xFF8000, 
        -30.00, 40.00, -50.00, 0xFF00FF, 
        -30.00, -40.00, -50.00, 0xFFFF00, 
        30.00, -40.00, -50.00, 0x00FFFF, 
        30.00, 40.00, -50.00, 0xFF8000,

        20.00, 30.00+100, 40.00, 0xFF00FF, 
        20.00, -30.00+100, 40.00, 0xFFFF00, 
        -20.00, -30.00+100, 40.00, 0x00FFFF, 
        -20.00, 30.00+100, 40.00, 0xFF8000, 
        -20.00, 30.00+100, -40.00, 0xFF00FF, 
        -20.00, -30.00+100, -40.00, 0xFFFF00, 
        20.00, -30.00+100, -40.00, 0x00FFFF, 
        20.00, 30.00+100, -40.00, 0xFF8000
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

    _3D_ppLink_add(ddat, 0+8, 3, 1+8, 3+8, 7+8);
    _3D_ppLink_add(ddat, 1+8, 2, 2+8, 6+8);
    _3D_ppLink_add(ddat, 2+8, 2, 3+8, 5+8);
    _3D_ppLink_add(ddat, 3+8, 1, 4+8);
    _3D_ppLink_add(ddat, 4+8, 2, 5+8, 7+8);
    _3D_ppLink_add(ddat, 5+8, 1, 6+8);
    _3D_ppLink_add(ddat, 6+8, 1, 7+8);

    //初始转角
    // ddat->raxyz[0] = _3D_PI/8;
    // ddat->raxyz[1] = _3D_PI/2;
    // ddat->raxyz[2] = -_3D_PI/4;

    while(1)
    {
        //
        // PRINT_CLEAR();

        _3D_angle_to_xyz(ddat);
        _3D_draw(VIEW_X_SIZE/2, VIEW_Y_SIZE/2, ddat);
        
        //
        PRINT_EN();

        // ddat->raxyz[0] += _3D_PI/16;
        // ddat->raxyz[1] += _3D_PI/16;
        ddat->raxyz[2] += _3D_PI/16;

        // if(scanf("%s", input))
        // {
        //     if(input[0] == '1')
        //         ddat->raxyz[0] += _3D_PI/16;
        //     else if(input[0] == 'q')
        //         ddat->raxyz[0] -= _3D_PI/16;

        //     else if(input[0] == '2')
        //         ddat->raxyz[1] += _3D_PI/16;
        //     else if(input[0] == 'w')
        //         ddat->raxyz[1] -= _3D_PI/16;

        //     else if(input[0] == '3')
        //         ddat->raxyz[2] += _3D_PI/16;
        //     else if(input[0] == 'e')
        //         ddat->raxyz[2] -= _3D_PI/16;

        //     else if(input[0] == 'r')
        //     {
        //         memcpy(ddat->array, ddat->arrayCopy, ddat->memSize);
        //         memset(ddat->out, 0, ddat->pointNum*2*sizeof(int));
        //         memset(ddat->raxyz, 0, 3*sizeof(double));
        //     }
        // }

        usleep(100000);
    }
}


