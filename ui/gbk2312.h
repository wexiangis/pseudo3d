
/*
 *  根据utf8字符串获取文字点阵
 */
#ifndef _GBK2312_H
#define _GBK2312_H

int gbk_getArrayByUtf8(unsigned char *utf8_code, unsigned char *buf, unsigned int *bufLen, int type);
int gbk_getStringWidthByUtf8(unsigned char *utf8_code, int type);

#endif

