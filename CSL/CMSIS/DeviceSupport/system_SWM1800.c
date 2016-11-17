/****************************************************************************************************************************************** 
* 文件名称:	system_SWM1800.c
* 功能说明:	SWM1800单片机的时钟设置
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
#include <stdint.h>
#include "SWM1800.h"


uint32_t SystemCoreClock  = __HSI;   				//System Clock Frequency (Core Clock)
uint32_t CyclesPerUs      = (__HSI / 1000000); 		//Cycles per micro second


/****************************************************************************************************************************************** 
* 函数名称: 
* 功能说明: This function is used to update the variable SystemCoreClock and must be called whenever the core clock is changed
* 输    入: 
* 输    出: 
* 注意事项: 
******************************************************************************************************************************************/
void SystemCoreClockUpdate(void)    
{
	if(SYS->CLKSEL & SYS_CLKSEL_SYS_Msk)			//SYS_CLK  <= HFCK
	{
		if(SYS->CLKSEL & SYS_CLKSEL_HFCK_Msk)			//HFCK <= HRC/4
		{
			if(SYS->HRCCR & SYS_HRCCR_DBL_Msk)				//HRC = 48MHz
			{
				SystemCoreClock = __HSI*2/4;
			}
			else											//HRC = 24MHz
			{
				SystemCoreClock =  __HSI/4;
			}
		}
		else											//HFCK <= HRC
		{
			if(SYS->HRCCR & SYS_HRCCR_DBL_Msk)				//HRC = 48MHz
			{
				SystemCoreClock = __HSI*2;
			}
			else											//HRC = 24MHz
			{
				SystemCoreClock = __HSI;
			}
		}
	}
	else											//SYS_CLK  <= LFCK
	{
		if(SYS->LRCCR & SYS_CLKSEL_LFCK_Msk)			//LFCK <= XTAL
		{
			SystemCoreClock = __HSE;
		}
		else											//LFCK <= LRC
		{
			SystemCoreClock = __LSI;
		}
	}
}

static void switchToRC24MHz(void);
static void switchToRC6MHz(void);
static void switchToRC48MHz(void);
static void switchToRC12MHz(void);
static void switchToRC32KHz(void);
static void switchToXTAL(void);
/****************************************************************************************************************************************** 
* 函数名称: 
* 功能说明: The necessary initializaiton of systerm
* 输    入: 
* 输    出: 
* 注意事项: 
******************************************************************************************************************************************/
void SystemInit(void)
{
	uint32_t i;
	
	SYS->CLKEN |= (1 << SYS_CLKEN_OSC_Pos);
	
	switch(SYS_CLK)
	{
		case SYS_CLK_24MHz:			//0 内部高频24MHz RC振荡器
			if(SYS->CLKSEL & SYS_CLKSEL_SYS_Msk)	//当前时钟是高频RC，修改高频RC时钟频率时需要先切到一个稳定时钟源
			{
				switchToRC32KHz();
			}
			switchToRC24MHz();
			break;
		
		case SYS_CLK_6MHz:			//1 内部高频 6MHz RC振荡器
			if(SYS->CLKSEL & SYS_CLKSEL_SYS_Msk)	//当前时钟是高频RC，修改高频RC时钟频率时需要先切到一个稳定时钟源
			{
				switchToRC32KHz();
			}
			switchToRC6MHz();
			break;
		
		case SYS_CLK_48MHz:			//2 内部高频48MHz RC振荡器
			if(SYS->CLKSEL & SYS_CLKSEL_SYS_Msk)	//当前时钟是高频RC，修改高频RC时钟频率时需要先切到一个稳定时钟源
			{
				switchToRC32KHz();
			}
			switchToRC48MHz();
			break;
		
		case SYS_CLK_12MHz:			//3 内部高频12MHz RC振荡器
			if(SYS->CLKSEL & SYS_CLKSEL_SYS_Msk)	//当前时钟是高频RC，修改高频RC时钟频率时需要先切到一个稳定时钟源
			{
				switchToRC32KHz();
			}
			switchToRC12MHz();
			break;
		
		case SYS_CLK_32KHz:			//4 内部低频32KHz RC振荡器
			if((SYS->CLKSEL & SYS_CLKSEL_SYS_Msk) == 0)
			{
				switchToRC24MHz();
			}
			switchToRC32KHz();
			break;
		
		case SYS_CLK_XTAL:			//5 外部XTAL晶体振荡器（2-30MHz）
			if((SYS->CLKSEL & SYS_CLKSEL_SYS_Msk) == 0)
			{
				switchToRC24MHz();
			}
			switchToXTAL();
			break;
	}
	
	for(i = 0;i <10000;i++);		//等待时钟稳定。。。
	
	SystemCoreClockUpdate();
}

static void switchToRC24MHz(void)
{
	SYS->HRCCR = (1 << SYS_HRCCR_EN_Pos) |
				 (0 << SYS_HRCCR_DBL_Pos);		//HRC = 24MHz
	
	SYS->CLKSEL &= ~SYS_CLKSEL_HFCK_Msk;		//HFCK  <=  HRC
	SYS->CLKSEL |= (1 << SYS_CLKSEL_SYS_Pos);	//SYS_CLK  <= HFCK
}

static void switchToRC6MHz(void)
{
	SYS->HRCCR = (1 << SYS_HRCCR_EN_Pos) |
				 (0 << SYS_HRCCR_DBL_Pos);		//HRC = 24MHz
	
	SYS->CLKSEL |= (1 << SYS_CLKSEL_HFCK_Pos);	//HFCK  <=  HRC/4
	SYS->CLKSEL |= (1 << SYS_CLKSEL_SYS_Pos);	//SYS_CLK  <= HFCK
}

static void switchToRC48MHz(void)
{
	SYS->HRCCR = (1 << SYS_HRCCR_EN_Pos) |
				 (1 << SYS_HRCCR_DBL_Pos);		//HRC = 48MHz		
	
	SYS->CLKSEL &= ~SYS_CLKSEL_HFCK_Msk;		//HFCK  <=  HRC
	SYS->CLKSEL |= (1 << SYS_CLKSEL_SYS_Pos);	//SYS_CLK  <= HFCK
}

static void switchToRC12MHz(void)
{
	SYS->HRCCR = (1 << SYS_HRCCR_EN_Pos) |
				 (1 << SYS_HRCCR_DBL_Pos);		//HRC = 48MHz
	
	SYS->CLKSEL |= (1 << SYS_CLKSEL_HFCK_Pos);	//HFCK  <=  HRC/4
	SYS->CLKSEL |= (1 << SYS_CLKSEL_SYS_Pos);	//SYS_CLK  <= HFCK
}

static void switchToRC32KHz(void)
{
	SYS->LRCCR = (1 << SYS_LRCCR_EN_Pos);
	
	SYS->CLKSEL &= ~(1 << SYS_CLKSEL_LFCK_Pos);	//LFCK  <=  LRC
	SYS->CLKSEL &= ~(1 << SYS_CLKSEL_SYS_Pos);	//SYS_CLK  <= LFCK
}

static void switchToXTAL(void)
{
	uint32_t i;
	
	PORT_Init(PORTC, PIN1, PORTC_PIN1_XTAL_IN, 0);
	PORT_Init(PORTC, PIN0, PORTC_PIN0_XTAL_OUT, 0);
	
	SYS->XTALCR = (1 << SYS_XTALCR_EN_Pos);
	
	for(i = 0; i < 22118; i++);
	
	SYS->CLKSEL |= (1 << SYS_CLKSEL_LFCK_Pos);	//LFCK  <=  XTAL
	SYS->CLKSEL &= ~(1 << SYS_CLKSEL_SYS_Pos);	//SYS_CLK  <= LFCK
}
