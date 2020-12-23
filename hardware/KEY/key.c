/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, temp_duan, China, DY.
**                           All Rights Reserved
**
**                           By(德阳科派科技有限公司)
**                             http://www.kpiot.top
**
**----------------------------------文件信息------------------------------------
** 文件名称: key.c
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
 
/********************************End of File************************************/

#include "key.h"



/*******************************************************************************
** 函数名称: KEY_Init
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
void KEY_Init(void) //IO初始化
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 										
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
