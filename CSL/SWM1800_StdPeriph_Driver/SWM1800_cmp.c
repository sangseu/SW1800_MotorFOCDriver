/****************************************************************************************************************************************** 
* 文件名称:	SWM1800_cmp.c
* 功能说明:	SWM1800单片机的CMP比较器功能驱动库
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
#include "SWM1800_cmp.h"


/****************************************************************************************************************************************** 
* 函数名称:	CMP_Init()
* 功能说明:	CMP模拟比较器初始化
* 输    入: uint32_t CMPx		指定要设置的比较器，有效值包括CMP0、CMP1、CMP2
*			uint32_t inp_ex		比较器正输入端IN+选择，1 外部CMPxP引脚    0 内部VREF参考电压
*			uint32_t hys_en		比较器输出迟滞使能，   1 开启迟滞特性     0 关闭迟滞特性
*			uint32_t int_en		比较器输出变化中断使能		 	
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void CMP_Init(uint32_t CMPx, uint32_t inp_ex, uint32_t hys_en, uint32_t int_en)
{
	switch(CMPx)
	{
	case CMP0:
		SYS->CMPCR &= ~(SYS_CMPCR_CMP0INP_Msk | SYS_CMPCR_CMP0HYS_Msk);
		SYS->CMPCR |= (inp_ex << SYS_CMPCR_CMP0INP_Pos) | (hys_en << SYS_CMPCR_CMP0HYS_Pos);
		CMP_INTClr(CMP0);
		SYS->CMPSR &= ~SYS_CMPSR_CMP0IE_Msk;
		SYS->CMPSR |= (int_en << SYS_CMPSR_CMP0IE_Pos);
		break;
	
	case CMP1:
		SYS->CMPCR &= ~(SYS_CMPCR_CMP1INP_Msk | SYS_CMPCR_CMP1HYS_Msk);
		SYS->CMPCR |= (inp_ex << SYS_CMPCR_CMP1INP_Pos) | (hys_en << SYS_CMPCR_CMP1HYS_Pos);
		CMP_INTClr(CMP1);
		SYS->CMPSR &= ~SYS_CMPSR_CMP1IE_Msk;
		SYS->CMPSR |= (int_en << SYS_CMPSR_CMP1IE_Pos);
		break;
	
	case CMP2:
		SYS->CMPCR &= ~(SYS_CMPCR_CMP2INP_Msk | SYS_CMPCR_CMP2HYS_Msk);
		SYS->CMPCR |= (inp_ex << SYS_CMPCR_CMP2INP_Pos) | (hys_en << SYS_CMPCR_CMP2HYS_Pos);
		CMP_INTClr(CMP2);
		SYS->CMPSR &= ~SYS_CMPSR_CMP2IE_Msk;
		SYS->CMPSR |= (int_en << SYS_CMPSR_CMP2IE_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称:	CMP_Open()
* 功能说明:	开启比较器
* 输    入: uint32_t CMPx		指定要设置的比较器，有效值包括CMP0、CMP1、CMP2
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void CMP_Open(uint32_t CMPx)
{
	switch(CMPx)
	{
	case CMP0:
		SYS->CMPCR |= (0x01 << SYS_CMPCR_CMP0EN_Pos);
		break;
	
	case CMP1:
		SYS->CMPCR |= (0x01 << SYS_CMPCR_CMP1EN_Pos);
		break;
	
	case CMP2:
		SYS->CMPCR |= (0x01 << SYS_CMPCR_CMP2EN_Pos);
		break;
	}		
}

/****************************************************************************************************************************************** 
* 函数名称:	CMP_Close()
* 功能说明:	关闭比较器
* 输    入: uint32_t CMPx		指定要设置的比较器，有效值包括CMP0、CMP1、CMP2
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void CMP_Close(uint32_t CMPx)
{
	switch(CMPx)
	{
	case CMP0:
		SYS->CMPCR &= ~(0x01 << SYS_CMPCR_CMP0EN_Pos);
		break;
	
	case CMP1:
		SYS->CMPCR &= ~(0x01 << SYS_CMPCR_CMP1EN_Pos);
		break;
	
	case CMP2:
		SYS->CMPCR &= ~(0x01 << SYS_CMPCR_CMP2EN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称:	CMP_Output()
* 功能说明:	比较器输出
* 输    入: uint32_t CMPx		指定要设置的比较器，有效值包括CMP0、CMP1、CMP2
* 输    出: uint32_t			0 CMPxN电平高于CMPxP电平    1 CMPxN电平低于CMPxP电平    0xFF 调用函数时参数错误
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t CMP_Output(uint32_t CMPx)
{
	switch(CMPx)
	{
	case CMP0:
		return (SYS->CMPSR & SYS_CMPSR_CMP0OUT_Msk) ? 1 : 0;
	
	case CMP1:
		return (SYS->CMPSR & SYS_CMPSR_CMP1OUT_Msk) ? 1 : 0;
	
	case CMP2:
		return (SYS->CMPSR & SYS_CMPSR_CMP2OUT_Msk) ? 1 : 0;
	}
	
	return 0xFF;
}

/****************************************************************************************************************************************** 
* 函数名称:	CMP_SetVRef()
* 功能说明:	设置内部参考电压
* 输    入: uint32_t volt		内部参考电压，有效值有CMP_VREF_0V30、CMP_VREF_0V45、... ... 、CMP_VREF_2V55
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void CMP_SetVRef(uint32_t volt)
{
	SYS->CMPCR &= ~SYS_CMPCR_VREF_Msk;
	SYS->CMPCR |= (volt << SYS_CMPCR_VREF_Pos);
}


/****************************************************************************************************************************************** 
* 函数名称:	CMP_INTEn()
* 功能说明:	比较器输出变化中断使能
* 输    入: uint32_t CMPx		指定要设置的比较器，有效值包括CMP0、CMP1、CMP2
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void CMP_INTEn(uint32_t CMPx)
{
	switch(CMPx)
	{
	case CMP0:
		SYS->CMPSR |= (0x01 << SYS_CMPSR_CMP0IE_Pos);
		break;
	
	case CMP1:
		SYS->CMPSR |= (0x01 << SYS_CMPSR_CMP1IE_Pos);
		break;
	
	case CMP2:
		SYS->CMPSR |= (0x01 << SYS_CMPSR_CMP2IE_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称:	CMP_INTDis()
* 功能说明:	比较器输出变化中断禁止
* 输    入: uint32_t CMPx		指定要设置的比较器，有效值包括CMP0、CMP1、CMP2
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void CMP_INTDis(uint32_t CMPx)
{
	switch(CMPx)
	{
	case CMP0:
		SYS->CMPSR &= ~(0x01 << SYS_CMPSR_CMP0IE_Pos);
		break;
	
	case CMP1:
		SYS->CMPSR &= ~(0x01 << SYS_CMPSR_CMP1IE_Pos);
		break;
	
	case CMP2:
		SYS->CMPSR &= ~(0x01 << SYS_CMPSR_CMP2IE_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称:	CMP_INTClr()
* 功能说明:	比较器输出变化中断标志清除
* 输    入: uint32_t CMPx		指定要设置的比较器，有效值包括CMP0、CMP1、CMP2
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void CMP_INTClr(uint32_t CMPx)
{
	switch(CMPx)
	{
	case CMP0:
		SYS->CMPSR = (SYS->CMPSR & 0xFF) | (0x01 << SYS_CMPSR_CMP0IF_Pos);
		break;
	
	case CMP1:
		SYS->CMPSR = (SYS->CMPSR & 0xFF) | (0x01 << SYS_CMPSR_CMP1IF_Pos);
		break;
	
	case CMP2:
		SYS->CMPSR = (SYS->CMPSR & 0xFF) | (0x01 << SYS_CMPSR_CMP2IF_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称:	CMP_INTStat()
* 功能说明:	比较器输出变化中断状态
* 输    入: uint32_t CMPx		指定要设置的比较器，有效值包括CMP0、CMP1、CMP2
* 输    出: uint32_t			1 比较器输出有变化    0 比较器输出无变化    0xFF 调用函数时参数错误
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t CMP_INTStat(uint32_t CMPx)
{
	switch(CMPx)
	{
	case CMP0:
		return (SYS->CMPSR & SYS_CMPSR_CMP0IF_Msk);
	
	case CMP1:
		return (SYS->CMPSR & SYS_CMPSR_CMP1IF_Msk);
	
	case CMP2:
		return (SYS->CMPSR & SYS_CMPSR_CMP2IF_Msk);
	}
	
	return 0xFF;
}
