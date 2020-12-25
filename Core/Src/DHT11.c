/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, temp_duan, China, DY.
**                           All Rights Reserved
**
**                           By(德阳科派科技有限公司)
**                             http://www.kpiot.top
**
**----------------------------------文件信息------------------------------------
** 文件名称: DHT11.c
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
#include "stm32f1xx.h"
#include "DHT11.h"
#include "stm32f1xx_hal_gpio.h"
#include "usart.h"
#include "main.h"
#include "delay.h"
#define MAXTIMINGS 85      
void DHT11_IO_IN(void)//温湿度模块输入函数
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = DHT11_dat_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DHT11_dat_GPIO_Port, &GPIO_InitStructure);
}

void DHT11_IO_OUT(void)//温湿度模块输出函数
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = DHT11_dat_Pin;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(DHT11_dat_GPIO_Port, &GPIO_InitStructure);
}

//复位DHT11
void DHT11_Rst(void)
{
    DHT11_IO_OUT(); //SET OUTPUT
    HAL_GPIO_WritePin(DHT11_dat_GPIO_Port,DHT11_dat_Pin, GPIO_PIN_RESET);
    delay_ms(20);;    //拉低至少18ms
    HAL_GPIO_WritePin(DHT11_dat_GPIO_Port,DHT11_dat_Pin, GPIO_PIN_SET);
    delay_us(40);     //主机拉高20~40us
}
//等待DHT11的回应
//返回1:未检测到DHT11的存在
//返回0:存在
uint8_t DHT11_Check(void)
{
    uint8_t retry = 0; //定义临时变量
    DHT11_IO_IN();//SET INPUT
    while (( HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) == 1) && retry < 100) //DHT11会拉低40~80us
    {
        retry++;
        delay_us(1);
    };
    if(retry >= 100)return 1;
    else retry = 0;
    while ((HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) == 0) && retry < 100) //DHT11拉低后会再次拉高40~80us
    {
        retry++;
        delay_us(1);
    };
    if(retry >= 100)return 1;
    return 0;
}
//从DHT11读取一个位
//返回值：1/0
uint8_t DHT11_Read_Bit(void)
{
    uint8_t retry = 0;
    while(( HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) == 1) && retry < 100) //等待变为低电平
    {
        retry++;
        delay_us(1);
    }
    retry = 0;
    while(( HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) == 0) && retry < 100) //等待变高电平
    {
        retry++;
        delay_us(1);
    }
    delay_us(40);//等待40us
    if( HAL_GPIO_ReadPin(DHT11_dat_GPIO_Port, DHT11_dat_Pin) ){
			
        return 1;
		}
    else{
			
        return 0;}
}
//从DHT11读取一个字节
//返回值：读到的数据
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

//从DHT11读取一次数据
//temp:温度值(范围:0~50°)
//humi:湿度值(范围:20%~90%)
//返回值：0,正常;1,读取失败
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
        for(i = 0; i < 5; i++) //读取40位数据
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
//初始化DHT11的IO口 DQ 同时检测DHT11的存在
//返回1:不存在
//返回0:存在
void DHT11_Init(void)
{
    DHT11_Rst();  //复位DHT11
    DHT11_Check();//等待DHT11的回应
}
