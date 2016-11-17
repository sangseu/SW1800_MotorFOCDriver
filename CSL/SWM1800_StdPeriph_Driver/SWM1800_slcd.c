/****************************************************************************************************************************************** 
* 文件名称: SWM1800_slcd.c
* 功能说明:	SWM1800单片机的Segment LCD功能驱动库
* 技术支持:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* 注意事项:
* 版本日期:	V1.0.0		2016年1月30日
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
#include "SWM1800_slcd.h"


/****************************************************************************************************************************************** 
* 函数名称:	SLCD_Init()
* 功能说明:	SLCD接口初始化
* 输    入: SLCD_TypeDef * SLCDx	指定要被设置的SLCD接口，有效值包括SLCD
*			SLCD_InitStructure * initStruct    包含SLCD接口相关设定值的结构体
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SLCD_Init(SLCD_TypeDef * SLCDx, SLCD_InitStructure * initStruct)
{
	switch((uint32_t)SLCDx)
	{
	case ((uint32_t)SLCD):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_SLCD_Pos);
		break;
	}
	
	SLCDx->CR = (1 << SLCD_CR_DRIVEN_Pos) 						|
				(1 << SLCD_CR_SCANEN_Pos) 						|
				(1 << SLCD_CR_DISP_Pos)   						|	//关闭显示
				(initStruct->Bias << SLCD_CR_BIAS_Pos) 			|
				(initStruct->Duty << SLCD_CR_DUTY_Pos) 			|
				(initStruct->FrameFreq << SLCD_CR_SCANFRQ_Pos) 	|
				(initStruct->DriveCurr << SLCD_CR_DRIVSEL_Pos) 	|
				(0 << SLCD_CR_KEYSCAN_Pos)					   	|	//按键扫描功能关闭
				((SystemCoreClock/1000000) << SLCD_CR_CLKDIV_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SLCD_Open()
* 功能说明:	开启显示
* 输    入: SLCD_TypeDef * SLCDx	指定要被设置的SLCD接口，有效值包括SLCD
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SLCD_Open(SLCD_TypeDef * SLCDx)
{
	SLCDx->CR &= ~SLCD_CR_DISP_Msk;
	SLCDx->CR |= (0 << SLCD_CR_DISP_Pos);	//开启显示
}

/****************************************************************************************************************************************** 
* 函数名称:	SLCD_Close()
* 功能说明:	关闭显示
* 输    入: SLCD_TypeDef * SLCDx	指定要被设置的SLCD接口，有效值包括SLCD
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SLCD_Close(SLCD_TypeDef * SLCDx)
{
	SLCDx->CR &= ~SLCD_CR_DISP_Msk;
	SLCDx->CR |= (1 << SLCD_CR_DISP_Pos);	//关闭显示
}

/****************************************************************************************************************************************** 
* 函数名称:	SLCD_Clear()
* 功能说明:	清除显示，所有字段都不亮
* 输    入: SLCD_TypeDef * SLCDx	指定要被设置的SLCD接口，有效值包括SLCD
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SLCD_Clear(SLCD_TypeDef * SLCDx)
{
	SLCD->DATA[0] = 0x00000000;
	SLCD->DATA[1] = 0x00000000;
	SLCD->DATA[2] = 0x00000000;
	SLCD->DATA[3] = 0x00000000;
}

/****************************************************************************************************************************************** 
* 函数名称:	SLCD_AllOn()
* 功能说明:	全屏显示，所有字段都点亮
* 输    入: SLCD_TypeDef * SLCDx	指定要被设置的SLCD接口，有效值包括SLCD
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SLCD_AllOn(SLCD_TypeDef * SLCDx)
{
	SLCD->DATA[0] = 0xFFFFFFFF;
	SLCD->DATA[1] = 0xFFFFFFFF;
	SLCD->DATA[2] = 0xFFFFFFFF;
	SLCD->DATA[3] = 0xFFFFFFFF;
}
