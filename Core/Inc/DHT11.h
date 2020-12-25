#ifndef __DHT11_H
#define __DHT11_H
#include "main.h"


void DHT11_IO_OUT(void);
void DHT11_IO_IN(void);
void DHT11_Init(void);
uint8_t   DHT11_Read_Data(uint8_t *temp,uint8_t *humi);
uint8_t   DHT11_Read_Byte(void);
uint8_t   DHT11_Read_Bit(void);
uint8_t   DHT11_Check(void);
void DHT11_Rst(void);

#endif

