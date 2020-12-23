/********************************End of File************************************//********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, temp_duan, China, DY.
**                           All Rights Reserved
**
**                           By(德阳科派科技有限公司)
**                             http://www.kpiot.top
**
**----------------------------------文件信息------------------------------------
** 文件名称: usart.c
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
#include "usart.h"



/*******************************************************************************
** 函数名称: fputc
** 功能描述:
** 参数说明: ch: [输入/出]
**			 f: [输入/出]
** 返回说明: None
** 创建人员: temp_duan
** 创建日期: 2019-07-09
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
int fputc(int ch, FILE *f)
{
    /* 等待发送完毕 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    /* 发送一个字节数据到USART1 */
    USART_SendData(USART1, (uint8_t) ch);
    return (ch);
}


/*******************************************************************************
** 函数名称: fgetc
** 功能描述:
** 参数说明: f: [输入/出]
** 返回说明: None
** 创建人员: temp_duan
** 创建日期: 2019-07-09
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
int fgetc(FILE *f)
{
    /* 等待串口1输入数据 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    return (int)USART_ReceiveData(USART1);

}

/*******************************************************************************
** 函数名称: usart_init
** 功能描述:
** 参数说明: val_1: [输入/出]
** 返回说明: None
** 创建人员: temp_duan
** 创建日期: 2019-07-09
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
void usart_init(u32 val_1)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;


    /* config USART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* USART1 GPIO config */
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 mode config */
    USART_InitStructure.USART_BaudRate = val_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* 使能串口1接收中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);



    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
** 函数名称: USART1_IRQHandler
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
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        u8 ch;
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);          /* Clear the USART Receive interrupt */
        ch = USART_ReceiveData(USART1);
        USART_SendData(USART1, ch);
    }

}
