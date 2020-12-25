/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, temp_duan, China, DY.
**                           All Rights Reserved
**
**                           By(�������ɿƼ����޹�˾)
**                             http://www.kpiot.top
**
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: DHT11.c
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
#include "stm32f1xx.h"
#include "DHT11.h"
#include "stm32f1xx_hal_gpio.h"
#include "usart.h"
#include "main.h"
#include "delay.h"
#define MAXTIMINGS 85      
void DHT11_IO_IN(void)//��ʪ��ģ�����뺯��
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = DHT11_dat_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DHT11_dat_GPIO_Port, &GPIO_InitStructure);
}

void DHT11_IO_OUT(void)//��ʪ��ģ���������
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = DHT11_dat_Pin;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(DHT11_dat_GPIO_Port, &GPIO_InitStructure);
}

//��λDHT11
void DHT11_Rst(void)
{
    DHT11_IO_OUT(); //SET OUTPUT
    HAL_GPIO_WritePin(DHT11_dat_GPIO_Port,DHT11_dat_Pin, GPIO_PIN_RESET);
    delay_ms(20);;    //��������18ms
    HAL_GPIO_WritePin(DHT11_dat_GPIO_Port,DHT11_dat_Pin, GPIO_PIN_SET);
    delay_us(40);     //��������20~40us
}
//�ȴ�DHT11�Ļ�Ӧ
//����1:δ��⵽DHT11�Ĵ���
//����0:����
uint8_t DHT11_Check(void)
{
    uint8_t retry = 0; //������ʱ����
    DHT11_IO_IN();//SET INPUT
    while (( HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) == 1) && retry < 100) //DHT11������40~80us
    {
        retry++;
        delay_us(1);
    };
    if(retry >= 100)return 1;
    else retry = 0;
    while ((HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) == 0) && retry < 100) //DHT11���ͺ���ٴ�����40~80us
    {
        retry++;
        delay_us(1);
    };
    if(retry >= 100)return 1;
    return 0;
}
//��DHT11��ȡһ��λ
//����ֵ��1/0
uint8_t DHT11_Read_Bit(void)
{
    uint8_t retry = 0;
    while(( HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) == 1) && retry < 100) //�ȴ���Ϊ�͵�ƽ
    {
        retry++;
        delay_us(1);
    }
    retry = 0;
    while(( HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) == 0) && retry < 100) //�ȴ���ߵ�ƽ
    {
        retry++;
        delay_us(1);
    }
    delay_us(40);//�ȴ�40us
    if( HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) ){
			
        return 1;
		}
    else{
			
        return 0;}
}
//��DHT11��ȡһ���ֽ�
//����ֵ������������
uint8_t DHT11_Read_Byte(void)
{
    uint8_t i=0, dat;
    dat = 0;
    for (i = 0; i < 8; i++)
    {
       dat <<= 1;
        dat |= DHT11_Read_Bit();
//			dat+=DHT11_Read_Bit();
	//		dat=dat*2;
	

    }
//	dat=dat/2;

    return dat;
}

//��DHT11��ȡһ������
//temp:�¶�ֵ(��Χ:0~50��)
//humi:ʪ��ֵ(��Χ:20%~90%)
//����ֵ��0,����;1,��ȡʧ��
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5]={0,0,0,0,0};
    uint8_t i=0;
    DHT11_Rst();
//		DHT11_IO_IN();//SET INPUT
//		HAL_GPIO_WritePin(DHT11_GPIO_Port,DHT11_Pin, GPIO_PIN_RESET);
//		if(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))
//		{
//			while(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin));
//			while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin));
		if(DHT11_Check()==0)
		{
        for(i = 0; i < 5; i++) //��ȡ40λ����
        {
            buf[i] = DHT11_Read_Byte();
				
			
        }
			
	//			while(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin));
	//			DHT11_IO_OUT(); 
	//			HAL_GPIO_WritePin(DHT11_GPIO_Port,DHT11_Pin, GPIO_PIN_SET);
        if((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
				
            *humi = buf[0];
            *temp = buf[2];
        }
    }
    else return 1;
    return 0;
	
}
//��ʼ��DHT11��IO�� DQ ͬʱ���DHT11�Ĵ���
//����1:������
//����0:����
void DHT11_Init(void)
{
    DHT11_Rst();  //��λDHT11
    DHT11_Check();//�ȴ�DHT11�Ļ�Ӧ
}
