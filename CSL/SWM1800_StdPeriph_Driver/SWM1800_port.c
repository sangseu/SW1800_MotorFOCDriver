/****************************************************************************************************************************************** 
* 文件名称: SWM1800_port.c
* 功能说明:	SWM1800单片机的端口引脚功能选择库函数
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
#include "SWM1800_port.h"


/****************************************************************************************************************************************** 
* 函数名称: PORT_Init()
* 功能说明:	端口引脚功能选择，可用的功能见"SWM1800_port.h"文件
* 输    入: PORT_TypeDef * PORTx	指定PORT端口，有效值包括PORTA、PORTB、PORTC、PORTD、PORTE	
*			uint32_t n		   		指定PORT引脚，有效值包括PIN0、PIN1、PIN2、... ... PIN14、PIN15
*			uint32_t func	   		指定端口引脚要设定的功能，其可取值见"SWM1800_port.h"文件
*			uint32_t digit_in_en	数字输入使能
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void PORT_Init(PORT_TypeDef * PORTx, uint32_t n, uint32_t func, uint32_t digit_in_en)
{
	
	if(func > 99)
	{
		if(n <= PIN5)
		{
			PORTx->FUNMUX0 &= ~(0x1F << (n*5));
			PORTx->FUNMUX0 |= (func-100) << (n*5);
		}
		else if(n <= PIN11)
		{
			PORTx->FUNMUX1 &= ~(0x1F << ((n-6)*5));
			PORTx->FUNMUX1 |= (func-100) << ((n-6)*5);
		}
		else if(n <= PIN15)
		{
			PORTx->FUNMUX2 &= ~(0x1F << ((n-12)*5));
			PORTx->FUNMUX2 |= (func-100) << ((n-12)*5);
		}
	}
	
	switch((uint32_t)PORTx)
	{
		case ((uint32_t)PORTA):
			if(n <= PIN7)
			{
				PORTG->PORTA_SEL1 &= ~(0x03 << (n*2));
				PORTG->PORTA_SEL1 |= (func > 99 ? 1 : func) << (n*2);
			}
			else if(n <= PIN11)
			{
				PORTG->PORTA_SEL2 &= ~(0x03 << ((n-8)*2));
				PORTG->PORTA_SEL2 |= (func > 99 ? 1 : func) << ((n-8)*2);
			}
			else if(n <= PIN15)
			{
				PORTG->PORTA_SEL2 &= ~(0x07 << (8+(n-12)*3));
				PORTG->PORTA_SEL2 |= (func > 99 ? 1 : func) << (8+(n-12)*3);
			}
			break;
			
		case ((uint32_t)PORTB):
			if(n <= PIN15)
			{
				PORTG->PORTB_SEL &= ~(0x03 << (n*2));
				PORTG->PORTB_SEL |= (func > 99 ? 1 : func) << (n*2);
			}
			break;
		
		case ((uint32_t)PORTC):
			if(n <= PIN3)
			{
				PORTG->PORTC_SEL &= ~(0x03 << (n*2));
				PORTG->PORTC_SEL |= (func > 99 ? 1 : func) << (n*2);
			}
			else if(n <= PIN7)
			{
				PORTG->PORTC_SEL &= ~(0x07 << (8+(n-4)*3));
				PORTG->PORTC_SEL |= (func > 99 ? 1 : func) << (8+(n-4)*3);
			}
			break;
		
		case ((uint32_t)PORTD):
			if(n <= PIN7)
			{
				PORTG->PORTD_SEL &= ~(0x03 << (n*2));
				PORTG->PORTD_SEL |= (func > 99 ? 1 : func) << (n*2);
			}
			break;
		
		case ((uint32_t)PORTE):
			if(n <= PIN7)
			{
				PORTG->PORTE_SEL &= ~(0x03 << (n*2));
				PORTG->PORTE_SEL |= (func > 99 ? 1 : func) << (n*2);
			}
			break;
	}
	
	PORTx->INEN &= ~(0x01 << n);
	PORTx->INEN |= (digit_in_en << n);
}
