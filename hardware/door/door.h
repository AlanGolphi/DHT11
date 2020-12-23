#ifndef  _DOOR_H
#define  _DOOR_H

#include "stm32f10x.h"

#define door GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)

void door_init(void);
#endif
