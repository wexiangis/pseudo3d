
/* 稍微精准的延时 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

void delayus(unsigned int us)
{
    struct timeval tv;
    tv.tv_sec = us / 1000000;
    tv.tv_usec = us % 1000000;
    select(0, NULL, NULL, NULL, &tv);
}

void delayms(unsigned int ms)
{
    delayus(ms * 1000);
}

long getTickUs(void)
{
    // struct timespec tp={0};
    struct timeval tv = {0};
    gettimeofday(&tv, NULL);
    return (long)(tv.tv_sec * 1000000u + tv.tv_usec);
}

void showTickUs(char enter)
{
    struct timeval tv = {0};
    int hour, min, sec, ms, us;
    gettimeofday(&tv, NULL);
    hour = (int)(tv.tv_sec / 3600 % 24 + 8);
    if (hour >= 24)
        hour -= 24;
    min = (int)(tv.tv_sec % 3600 / 60);
    sec = (int)(tv.tv_sec % 60);
    ms = (int)(tv.tv_usec / 1000);
    us = (int)(tv.tv_usec % 1000);
    if (enter)
        printf("[%02d:%02d:%02d:%03d:%03d]\r\n", hour, min, sec, ms, us);
    else
        printf("[%02d:%02d:%02d:%03d:%03d]", hour, min, sec, ms, us);
}
