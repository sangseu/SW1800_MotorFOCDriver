/****************************************************************************************************************************************** 
* 文件名称: SWM1800_irqmux.c
* 功能说明:	用于将外设中断连接到内核中断IRQ0--IRQ31
* 技术支持:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* 注意事项:
* 版本日期: V1.0.0		2016年1月30日
* 升级记录: 
*
*
*******************************************************************************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION 
* REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE 
* FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT 
* OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
* -ECTION WITH THEIR PRODUCTS.
*
* COPYRIGHT 2012 Synwit Technology 
*******************************************************************************************************************************************/
#include "SWM1800.h"
#include "SWM1800_irqmux.h"


/****************************************************************************************************************************************** 
* 函数名称: IRQ_Connect()
* 功能说明:	将外设中断连接到内核中断IRQ0--IRQ31
* 输    入: uint32_t periph_interrupt	指定外设中断，有效值见"SWM1800_irqmux.h"
*			uint32_t IRQn		       	要连接到的内核中断号，有效值IRQ0_IRQ、IRQ1_IRQ、 ... ... IRQ30_IRQ、IRQ31_IRQ，
*										由于IRQ16--IRQ31每一个IRQ上可以连接两个外设中断，所以需要“按位或”上IRQ_INT0和IRQ_INT1来区分，并且
*										IRQ30_IRQ | IRQ_INT0 == IRQ30_IRQ，所以“按位或”IRQ_INT0可以省去
*			uint32_t priority	       	中断优先级，有效值0、1、2、3，数值越小优先级越高
* 输    出: 无
* 注意事项: 举例，IRQ_Connect(IRQ0_15_GPIOA1, IRQ6_IRQ, 0)					将GPIOA端口引脚1上的中断连接到IRQ6内核中断
*
*                 IRQ_Connect(IRQ16_31_UART0, IRQ18_IRQ | IRQ_INT0, 0)		将UART0中断连接到IRQ18内核中断的中断0上，它还可简化成
*				  IRQ_Connect(IRQ16_31_UART0, IRQ18_IRQ, 0)
*
*				  IRQ_Connect(IRQ16_31_SDADC, IRQ24_IRQ | IRQ_INT1, 0)		将SDADC中断连接到IRQ24内核中断的中断1上
******************************************************************************************************************************************/
void IRQ_Connect(uint32_t periph_interrupt, uint32_t IRQn, uint32_t priority)
{
	uint32_t INT1 = 0;
	__IO uint32_t * IRQ_SRC = &IRQMUX->IRQ0_SRC;
	
	if(IRQn > 0xFF)
	{
		INT1 = 1;
		
		IRQn -= 0x100;
	}
	
	if(IRQn <= IRQ15_IRQ)
	{
		IRQ_SRC = &IRQMUX->IRQ0_SRC + (IRQn - IRQ0_IRQ);
		*IRQ_SRC = periph_interrupt;
	}
	else
	{
		IRQ_SRC = &IRQMUX->IRQ16_SRC + (IRQn - IRQ16_IRQ);
		
		if(INT1 == 1)
		{
			*IRQ_SRC &= ~(0x1F << 5);
			*IRQ_SRC |= (periph_interrupt << 5);
		}
		else
		{
			*IRQ_SRC &= ~(0x1F << 0);
			*IRQ_SRC |= (periph_interrupt << 0);
		}
	}
	
	NVIC_SetPriority((IRQn_Type)IRQn, priority);
	
	NVIC_EnableIRQ((IRQn_Type)IRQn);
}

/****************************************************************************************************************************************** 
* 函数名称: IRQ_Which()
* 功能说明:	IRQ16--IRQ31每一个IRQ上可以连接两个外设中断，此函数用于判断两个外设中断中的哪一个触发了中断请求
* 输    入: uint32_t IRQn	IRQ16_IRQ、IRQ17_IRQ、 ... ... IRQ30_IRQ、IRQ31_IRQ
* 输    出: uint32_t		IRQ_INT0 IRQn的INT0触发了中断    			IRQ_INT1 IRQn的INT1触发了中断
*							IRQ_BOTH IRQn的INT0和INT1都触发了中断		IRQ_NONE IRQn的INT0和INT1都未触发中断
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t IRQ_Which(uint32_t IRQn)
{
	uint32_t res = IRQ_NONE;
	__IO uint32_t * IRQ_IF = &IRQMUX->IRQ16_IF + (IRQn - IRQ16_IRQ);
	
	if((*IRQ_IF & 0x01) && (*IRQ_IF & 0x02))
		res = IRQ_BOTH;
	else if(*IRQ_IF & 0x01)
		res = IRQ_INT0;
	else if(*IRQ_IF & 0x02)
		res = IRQ_INT1;
	
	return res;
}
