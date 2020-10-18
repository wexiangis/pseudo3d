/*
 *  自制简易版示波器,图像输出到fb0
 */
#ifndef _WAVE_H_
#define _WAVE_H_

/*
 *  chn: 0~8
 */
void wave_load(int chn, short value);

void wave_refresh(void);

#endif 