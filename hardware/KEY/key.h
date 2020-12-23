#ifndef __KEY_H
#define __KEY_H	 
#include "stm32f10x.h"

#define KEY1    GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8) //读取按键1
#define KEY2    GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)//读取按键2
#define KEY3    GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)//读取按键3
#define KEY4    GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) //读取按键4
#define KEY5    GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) //读取按键5


void KEY_Init(void);//IO初始化

#endif

