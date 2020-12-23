#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define LED1_ON   GPIO_ResetBits(GPIOA,GPIO_Pin_5)
#define LED2_ON 	GPIO_ResetBits(GPIOA,GPIO_Pin_6)
#define LED3_ON 	GPIO_ResetBits(GPIOA,GPIO_Pin_7)
#define LED4_ON 	GPIO_ResetBits(GPIOB,GPIO_Pin_0)

#define LED1_OFF  GPIO_SetBits(GPIOA,GPIO_Pin_5)
#define LED2_OFF 	GPIO_SetBits(GPIOA,GPIO_Pin_6)
#define LED3_OFF 	GPIO_SetBits(GPIOA,GPIO_Pin_7)
#define LED4_OFF 	GPIO_SetBits(GPIOB,GPIO_Pin_0)

void LED_Init(void);//≥ı ºªØ



		 				    
#endif
