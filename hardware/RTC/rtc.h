#ifndef  _RTC_H
#define  _RTC_H


#include "stm32f10x.h"

#define HH 23
#define MM 59
#define SS 55

void RTC_Configuration(void);
void Time_Display(uint32_t TimeVar);


#endif
