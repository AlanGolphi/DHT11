/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, temp_duan, China, DY.
**                           All Rights Reserved
**
**                           By(德阳科派科技有限公司)
**                             http://www.kpiot.top
**
**----------------------------------文件信息------------------------------------
** 文件名称: led.c
** 创建人员: temp_duan
** 创建日期: 2019-07-09
** 文档描述: 
**
**----------------------------------版本信息------------------------------------
** 版本代号: V0.1
** 版本说明: 初始版本
**
**------------------------------------------------------------------------------
\********************************End of Head************************************/
 
#include "led.h"



/*******************************************************************************
** 函数名称: LED_Init
** 功能描述: 
** 参数说明: None
** 返回说明: None
** 创建人员: temp_duan
** 创建日期: 2019-07-09
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
		LED4_OFF;
}

