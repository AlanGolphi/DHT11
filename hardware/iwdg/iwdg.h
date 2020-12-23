#ifndef _IWDG_H
#define _IWDG_H

#include "stm32f10x.h"

void Iwdg_Init(unsigned char psc, unsigned short arr);
void Iwdg_Feed(void);


#endif
