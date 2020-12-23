#ifndef __DS18B20_H
#define __DS18B20_H

#include "stm32f10x.h"

#define IO_DS18B20 GPIO_Pin_13
#define GPIO_DS18B20 GPIOB

#define DS18B20_DQ_High GPIO_SetBits(GPIO_DS18B20,IO_DS18B20)
#define DS18B20_DQ_Low GPIO_ResetBits(GPIO_DS18B20,IO_DS18B20)

void DS18B20_IO_IN(void);
void DS18B20_IO_OUT(void);
u8 DS18B20_Read_Byte(void);
void DS18B20_Write_Byte(u8 dat);
void DS18B20_Reset(void);
double DS18B20_Get_wd(void);

#endif
















