#ifndef  _JDQ_H
#define  _JDQ_H

#include "stm32f10x.h"
#include "sys.h"


#define  Relay_OFF   GPIO_SetBits(GPIOB,GPIO_Pin_12)
#define  Relay_ON    GPIO_ResetBits(GPIOB,GPIO_Pin_12)

void Relay_init(void);

#endif

