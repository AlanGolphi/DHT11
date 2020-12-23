/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, temp_duan, China, DY.
**                           All Rights Reserved
**
**                           By(�������ɿƼ����޹�˾)
**                             http://www.kpiot.top
**
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: adcx.c
** ������Ա: temp_duan
** ��������: 2019-07-09
** �ĵ�����:
**
**----------------------------------�汾��Ϣ------------------------------------
** �汾����: V0.1
** �汾˵��: ��ʼ�汾
**
**------------------------------------------------------------------------------
\********************************End of Head************************************/

/********************************End of File************************************/
#include "adcx.h"
#include "usart.h"

__IO uint16_t ADC_ConvertedValue[macNOFCHANEL]= {0,0};

// �ֲ����������ڱ���ת�������ĵ�ѹֵ
float ADC_ConvertedValueLocal[macNOFCHANEL];

/**
  * @brief  ADC GPIO ��ʼ��
  * @param  ��
  * @retval ��
  */
static void ADCx_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // �� ADC IO�˿�ʱ��
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
  * @brief  ����ADC����ģʽ
  * @param  ��
  * @retval ��
  */
static void ADCx_Mode_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // ��DMAʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // ��ADCʱ��
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_ADC1, ENABLE );

    // ��λDMA������
    DMA_DeInit(macADC_DMA_CHANNEL);

    // ���� DMA ��ʼ���ṹ��
    // �����ַΪ��ADC ���ݼĴ�����ַ
    DMA_InitStructure.DMA_PeripheralBaseAddr = ( u32 ) ( & ( macADCx->DR ) );

    // �洢����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_ConvertedValue;

    // ����Դ��������
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;

    // ��������С��Ӧ�õ�������Ŀ�ĵصĴ�С
    DMA_InitStructure.DMA_BufferSize = macNOFCHANEL;

    // ����Ĵ���ֻ��һ������ַ���õ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    // �洢����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    // �������ݴ�СΪ���֣��������ֽ�
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

    // �ڴ����ݴ�СҲΪ���֣����������ݴ�С��ͬ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

    // ѭ������ģʽ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

    // DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ��DMAͨ��ʱ�����ȼ����ò�Ӱ��
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    // ��ֹ�洢�����洢��ģʽ����Ϊ�Ǵ����赽�洢��
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    // ��ʼ��DMA
    DMA_Init(macADC_DMA_CHANNEL, &DMA_InitStructure);

    // ʹ�� DMA ͨ��
    DMA_Cmd(macADC_DMA_CHANNEL, ENABLE);

    // ADC ģʽ����
    // ֻʹ��һ��ADC�����ڵ�ģʽ
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;

    // ɨ��ģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE ;

    // ����ת��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;

    // �����ⲿ����ת����������������
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

    // ת������Ҷ���
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

    // ת��ͨ������
    ADC_InitStructure.ADC_NbrOfChannel = macNOFCHANEL;

    // ��ʼ��ADC
    ADC_Init(macADCx, &ADC_InitStructure);

    // ����ADCʱ�ӣΪPCLK2��8��Ƶ����9MHz
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    // ����ADC ͨ����ת��˳��Ͳ���ʱ��
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_55Cycles5);


    // ʹ��ADC DMA ����
    ADC_DMACmd(macADCx, ENABLE);

    // ����ADC ������ʼת��
    ADC_Cmd(macADCx, ENABLE);

    // ��ʼ��ADC У׼�Ĵ���
    ADC_ResetCalibration(macADCx);
    // �ȴ�У׼�Ĵ�����ʼ�����
    while(ADC_GetResetCalibrationStatus(macADCx));

    // ADC��ʼУ׼
    ADC_StartCalibration(macADCx);
    // �ȴ�У׼���
    while(ADC_GetCalibrationStatus(macADCx));

    // ����û�в����ⲿ����������ʹ����������ADCת��
    ADC_SoftwareStartConvCmd(macADCx, ENABLE);
}

/**
  * @brief  ADC��ʼ��
  * @param  ��
  * @retval ��
  */
void ADCx_Init(void)
{
    ADCx_GPIO_Config();
    ADCx_Mode_Config();
}
/*********************************************END OF FILE**********************/


/*******************************************************************************
** ��������: AD_Read
** ��������:
** ����˵��: adc1: [����/��]
**			 		 adc2: [����/��]
** ����˵��: None
** ������Ա: temp_duan
** ��������: 2019-07-09
**------------------------------------------------------------------------------
** �޸���Ա:
** �޸�����:
** �޸�����:
**------------------------------------------------------------------------------
********************************************************************************/
void AD_Read(u8 *adc1, u8 *adc2)
{

    *adc1 =100 - ((float) ADC_ConvertedValue[0]/4096*3.3 *30);
    *adc2 =(float) ADC_ConvertedValue[1]/4096*3.3 *30;

}