/****************************************************************************************************************************************** 
* 文件名称: SWM1800_gpio.c
* 功能说明:	SWM1800单片机的通用输入输出功能驱动库
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
#include "SWM1800_gpio.h"


/****************************************************************************************************************************************** 
* 函数名称: GPIO_Init()
* 功能说明:	引脚初始化，包含引脚方向、上拉电阻、下拉电阻、开漏输出
* 输    入: GPIO_TypeDef * GPIOx	    指定GPIO端口，有效值包括GPIOA、GPIOB、GPIOC、GPIOD、GPIOE	
*			uint32_t n		       指定GPIO引脚，有效值包括PIN0、PIN1、PIN2、... ... PIN14、PIN15
*			uint32_t dir	       引脚方向，0 输入        1 输出
*			uint32_t pull_up	   上拉电阻，0 关闭上拉    1 开启上拉
*			uint32_t pull_down	   下拉电阻，0 关闭下拉    1 开启下拉
*			uint32_t open_drain	   开漏输出，0 推挽输出    1 开漏输出
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void GPIO_Init(GPIO_TypeDef * GPIOx, uint32_t n, uint32_t dir, uint32_t pull_up, uint32_t pull_down, uint32_t open_drain)
{
	PORT_TypeDef * PORTx = PORTA;
	
	switch((uint32_t)GPIOx)
	{
	case ((uint32_t)GPIOA):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_GPIOA_Pos);
		
		PORTx = PORTA;
		break;
	
	case ((uint32_t)GPIOB):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_GPIOB_Pos);
		
		PORTx = PORTB;
		break;

	case ((uint32_t)GPIOC):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_GPIOC_Pos);
		
		PORTx = PORTC;
		break;
		
	case ((uint32_t)GPIOD):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_GPIOD_Pos);
		
		PORTx = PORTD;
		break;
		
	case ((uint32_t)GPIOE):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_GPIOE_Pos);
		
		PORTx = PORTE;
		break;
	}
		
	if(dir == 1)
	{
		PORT_Init(PORTx, n, 0, 0);			//PORTP.PINn引脚配置为GPIO功能，数字输入关闭
		
		GPIOx->DIR |= (0x01 << n);
	}
	else
	{
		PORT_Init(PORTx, n, 0, 1);			//PORTP.PINn引脚配置为GPIO功能，数字输入开启
		
		GPIOx->DIR &= ~(0x01 << n);
	}
	
	if(pull_up == 1)
		PORTx->PULLU |= (0x01 << n);
	else
		PORTx->PULLU &= ~(0x01 << n);
	
	if(pull_down == 1)
		PORTx->PULLD |= (0x01 << n);
	else
		PORTx->PULLD &= ~(0x01 << n);
	
	if(open_drain == 1)
		PORTx->OPEND |= (0x01 << n);
	else
		PORTx->OPEND &= ~(0x01 << n);
}

/****************************************************************************************************************************************** 
* 函数名称: GPIO_SetBit()
* 功能说明:	将参数指定的引脚电平置高
* 输    入: GPIO_TypeDef * GPIOx	指示GPIO端口，有效值包括GPIOA、GPIOB、GPIOC、GPIOD、GPIOE	
*			uint32_t n		指示GPIO引脚，有效值包括PIN_0、PIN_1、PIN_2、... ... PIN_14、PIN_15
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void GPIO_SetBit(GPIO_TypeDef * GPIOx, uint32_t n)
{
	GPIOx->DATA |= (0x01 << n);
}

/****************************************************************************************************************************************** 
* 函数名称:	GPIO_ClrBit()
* 功能说明:	将参数指定的引脚电平置低
* 输    入: GPIO_TypeDef * GPIOx	指示GPIO端口，有效值包括GPIOA、GPIOB、GPIOC、GPIOD、GPIOE	
*			uint32_t n		指示GPIO引脚，有效值包括PIN_0、PIN_1、PIN_2、... ... PIN_14、PIN_15
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void GPIO_ClrBit(GPIO_TypeDef * GPIOx, uint32_t n)
{
	GPIOx->DATA &= ~(0x01 << n);
}

/****************************************************************************************************************************************** 
* 函数名称: GPIO_InvBit()
* 功能说明:	将参数指定的引脚电平反转
* 输    入: GPIO_TypeDef * GPIOx	指示GPIO端口，有效值包括GPIOA、GPIOB、GPIOC、GPIOD、GPIOE	
*			uint32_t n		指示GPIO引脚，有效值包括PIN_0、PIN_1、PIN_2、... ... PIN_14、PIN_15
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void GPIO_InvBit(GPIO_TypeDef * GPIOx, uint32_t n)
{
	GPIOx->DATA ^= (0x01 << n);
}

/****************************************************************************************************************************************** 
* 函数名称:	GPIO_GetBit()
* 功能说明:	读取参数指定的引脚的电平状态
* 输    入: GPIO_TypeDef * GPIOx	指示GPIO端口，有效值包括GPIOA、GPIOB、GPIOC、GPIOD、GPIOE	
*			uint32_t n		指示GPIO引脚，有效值包括PIN_0、PIN_1、PIN_2、... ... PIN_14、PIN_15
* 输    出: 参数指定的引脚的电平状态	0 低电平	1 高电平
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t GPIO_GetBit(GPIO_TypeDef * GPIOx, uint32_t n)
{	
	return ((GPIOx->DATA >> n) & 0x01);
}

/****************************************************************************************************************************************** 
* 函数名称: GPIO_SetBits()
* 功能说明: 将参数指定的从n开始的w位连续引脚的电平置高
* 输    入: GPIO_TypeDef * GPIOx	指示GPIO端口，有效值包括GPIOA、GPIOB、GPIOC、GPIOD、GPIOE	
*			uint32_t n		指示GPIO引脚，有效值包括PIN_0、PIN_1、PIN_2、... ... PIN_14、PIN_15
*			uint32_t w		指定要将引脚电平置高的引脚的个数
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void GPIO_SetBits(GPIO_TypeDef * GPIOx, uint32_t n, uint32_t w)
{
	uint32_t bits;
	
	bits = 0xFFFF >> (16 - w);
	
	GPIOx->DATA |= (bits << n);
}

/****************************************************************************************************************************************** 
* 函数名称:	GPIO_ClrBits()
* 功能说明: 将参数指定的从n开始的w位连续引脚的电平置低
* 输    入: GPIO_TypeDef * GPIOx	指示GPIO端口，有效值包括GPIOA、GPIOB、GPIOC、GPIOD、GPIOE	
*			uint32_t n		指示GPIO引脚，有效值包括PIN_0、PIN_1、PIN_2、... ... PIN_14、PIN_15
*			uint32_t w		指定要将引脚电平置低的引脚的个数
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void GPIO_ClrBits(GPIO_TypeDef * GPIOx, uint32_t n, uint32_t w)
{
	uint32_t bits;
	
	bits = 0xFFFF >> (16 - w);
	
	GPIOx->DATA &= ~(bits << n);
}

/****************************************************************************************************************************************** 
* 函数名称: GPIO_InvBits()
* 功能说明: 将参数指定的从n开始的w位连续引脚的电平反转
* 输    入: GPIO_TypeDef * GPIOx	指示GPIO端口，有效值包括GPIOA、GPIOB、GPIOC、GPIOD、GPIOE	
*			uint32_t n		指示GPIO引脚，有效值包括PIN_0、PIN_1、PIN_2、... ... PIN_14、PIN_15
*			uint32_t w		指定要将引脚电平反转的引脚的个数
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void GPIO_InvBits(GPIO_TypeDef * GPIOx, uint32_t n, uint32_t w)
{
	uint32_t bits;
	
	bits = 0xFFFF >> (16 - w);
	
	GPIOx->DATA ^= (bits << n);
}

/****************************************************************************************************************************************** 
* 函数名称:	GPIO_GetBits()
* 功能说明: 读取参数指定的从n开始的w位连续引脚的电平状态
* 输    入: GPIO_TypeDef * GPIOx	指示GPIO端口，有效值包括GPIOA、GPIOB、GPIOC、GPIOD、GPIOE	
*			uint32_t n		指示GPIO引脚，有效值包括PIN_0、PIN_1、PIN_2、... ... PIN_14、PIN_15
*			uint32_t w		指定要将引脚电平置高的引脚的个数
* 输    出: 参数指定的从n开始的w位连续引脚的电平状态	0 低电平	1 高电平
*			返回值的第0位表示引脚n的电平状态、返回值的第1位表示引脚n+1的电平状态... ...返回值的第w位表示引脚n+w的电平状态
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t GPIO_GetBits(GPIO_TypeDef * GPIOx, uint32_t n, uint32_t w)
{
	uint32_t bits;
	
   	bits = 0xFFFF >> (16 - w);
	
	return ((GPIOx->DATA >> n) & bits);
}
