/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2019, temp_duan, China, DY.
**                           All Rights Reserved
**
**                           By(德阳科派科技有限公司)
**                             http://www.kpiot.top
**
**----------------------------------文件信息------------------------------------
** 文件名称: sys.c
** 创建人员: temp_duan
** 创建日期: 2019-07-21
** 文档描述: 
**
**----------------------------------版本信息------------------------------------
** 版本代号: V0.1
** 版本说明: 初始版本
**
**------------------------------------------------------------------------------
\********************************End of Head************************************/
#include "sys.h"
void WFI_SET(void)
{
    __ASM volatile("wfi");
}
//关闭所有中断
void INTX_DISABLE(void)
{
    __ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
    __ASM volatile("cpsie i");
}

#if 0
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr)
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}
#endif
