#ifndef  _BEEP_H
#define  _BEEP_H

#include "stm32f10x.h"

#define BEEP_ON 	GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define BEEP_OFF 	GPIO_ResetBits(GPIOB,GPIO_Pin_9)

void beep_init(void);
#endif
