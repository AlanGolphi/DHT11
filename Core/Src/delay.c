//单片机头文件
#include "stm32f1xx.h"

//delay头文件
#include "delay.h"




/*
************************************************************
*	函数名称：	Delay_Init
*
*	函数功能：	systick初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：
************************************************************
*/
void delay_us(uint32_t nus)
{
 uint32_t temp;
 SysTick->LOAD = 9*nus;
 SysTick->VAL=0X00;//?????
 SysTick->CTRL=0X01;//??,???????,???????
 do
 {
  temp=SysTick->CTRL;//????????
 }while((temp&0x01)&&(!(temp&(1<<16))));//??????
     SysTick->CTRL=0x00; //?????
    SysTick->VAL =0X00; //?????
}
void delay_ms(uint16_t nms)
{
 uint32_t temp;
 SysTick->LOAD = 9000*nms;
 SysTick->VAL=0X00;//?????
 SysTick->CTRL=0X01;//??,???????,???????
 do
 {
  temp=SysTick->CTRL;//????????
 }while((temp&0x01)&&(!(temp&(1<<16))));//??????
    SysTick->CTRL=0x00; //?????
    SysTick->VAL =0X00; //?????
}
