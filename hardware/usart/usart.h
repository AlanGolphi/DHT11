#ifndef   _USART_H
#define   _USART_H
#include "stm32f10x.h"
#include <stdio.h>

#define  RINGBUFF_LEN  		200  	//定义最大接收字节数 200

/*  设置三个串口的波特率  */
void usart_init(u32 val_1);


#endif

