
/* 稍微精准的延时 */
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