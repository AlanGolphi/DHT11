/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, temp_duan, China, DY.
**                           All Rights Reserved
**
**                           By(德阳科派科技有限公司)
**                             http://www.kpiot.top
**
**----------------------------------文件信息------------------------------------
** 文件名称: adcx.c
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
#include "adcx.h"
#include "usart.h"

__IO uint16_t ADC_ConvertedValue[macNOFCHANEL]= {0,0};

// 局部变量，用于保存转换计算后的电压值
float ADC_ConvertedValueLocal[macNOFCHANEL];

/**
  * @brief  ADC GPIO 初始化
  * @param  无
  * @retval 无
  */
static void ADCx_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 打开 ADC IO端口时钟
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOA, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/**
  * @brief  配置ADC工作模式
  * @param  无
  * @retval 无
  */
static void ADCx_Mode_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // 打开DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // 打开ADC时钟
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_ADC1, ENABLE );

    // 复位DMA控制器
    DMA_DeInit(macADC_DMA_CHANNEL);

    // 配置 DMA 初始化结构体
    // 外设基址为：ADC 数据寄存器地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = ( u32 ) ( & ( macADCx->DR ) );

    // 存储器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_ConvertedValue;

    // 数据源来自外设
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;

    // 缓冲区大小，应该等于数据目的地的大小
    DMA_InitStructure.DMA_BufferSize = macNOFCHANEL;

    // 外设寄存器只有一个，地址不用递增
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    // 存储器地址递增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    // 外设数据大小为半字，即两个字节
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

    // 内存数据大小也为半字，跟外设数据大小相同
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

    // 循环传输模式
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

    // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    // 禁止存储器到存储器模式，因为是从外设到存储器
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    // 初始化DMA
    DMA_Init(macADC_DMA_CHANNEL, &DMA_InitStructure);

    // 使能 DMA 通道
    DMA_Cmd(macADC_DMA_CHANNEL, ENABLE);

    // ADC 模式配置
    // 只使用一个ADC，属于单模式
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;

    // 扫描模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE ;

    // 连续转换模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;

    // 不用外部触发转换，软件开启即可
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

    // 转换结果右对齐
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

    // 转换通道个数
    ADC_InitStructure.ADC_NbrOfChannel = macNOFCHANEL;

    // 初始化ADC
    ADC_Init(macADCx, &ADC_InitStructure);

    // 配置ADC时钟Ｎ狿CLK2的8分频，即9MHz
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    // 配置ADC 通道的转换顺序和采样时间
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_55Cycles5);


    // 使能ADC DMA 请求
    ADC_DMACmd(macADCx, ENABLE);

    // 开启ADC ，并开始转换
    ADC_Cmd(macADCx, ENABLE);

    // 初始化ADC 校准寄存器
    ADC_ResetCalibration(macADCx);
    // 等待校准寄存器初始化完成
    while(ADC_GetResetCalibrationStatus(macADCx));

    // ADC开始校准
    ADC_StartCalibration(macADCx);
    // 等待校准完成
    while(ADC_GetCalibrationStatus(macADCx));

    // 由于没有采用外部触发，所以使用软件触发ADC转换
    ADC_SoftwareStartConvCmd(macADCx, ENABLE);
}

/**
  * @brief  ADC初始化
  * @param  无
  * @retval 无
  */
void ADCx_Init(void)
{
    ADCx_GPIO_Config();
    ADCx_Mode_Config();
}
/*********************************************END OF FILE**********************/


/*******************************************************************************
** 函数名称: AD_Read
** 功能描述:
** 参数说明: adc1: [输入/出]
**			 		 adc2: [输入/出]
** 返回说明: None
** 创建人员: temp_duan
** 创建日期: 2019-07-09
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
void AD_Read(u8 *adc1, u8 *adc2)
{

    *adc1 =100 - ((float) ADC_ConvertedValue[0]/4096*3.3 *30);
    *adc2 =(float) ADC_ConvertedValue[1]/4096*3.3 *30;

}
