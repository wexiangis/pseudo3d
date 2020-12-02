#ifndef _DELAYUS_H_
#define _DELAYUS_H_

void delayus(unsigned int us);
void delayms(unsigned int ms);

//自动校准的延时
#define DELAY_US_INIT \
    long _tick1 = 0, _tick2 = 0;
#define DELAY_US(us)                             \
    _tick2 = getTickUs();                        \
    if (_tick2 > _tick1 && _tick2 - _tick1 < us) \
        delayus(us - (_tick2 - _tick1));         \
    _tick1 = getTickUs();

long getTickUs(void);
void showTickUs(char enter);

#endif
