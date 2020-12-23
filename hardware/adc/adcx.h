#ifndef _ADCX_H
#define _ADCX_H

#include "stm32f10x.h"

// 转换通道个数
#define    macNOFCHANEL											2



// ADC1 对应 DMA1通道1，ADC3对应DMA2通道5，ADC2没有DMA功能
#define    macADCx                          ADC1
#define    macADC_DMA_CHANNEL               DMA1_Channel1


/**************************函数声明********************************/
void               ADCx_Init                               (void);
void AD_Read(u8 *adc1, u8 *adc2);

#endif

