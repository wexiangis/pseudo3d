
/* 稍微精准的延时 */
#include <stdlib.h>
#include <sys/time.h>

void delayus(unsigned int us)
{
    struct timeval delay;
    delay.tv_sec = us / 1000000;
    delay.tv_usec = us % 1000000;
    select(0, NULL, NULL, NULL, &delay);
}

void delayms(unsigned int ms)
{
    delayus(ms * 1000);
}