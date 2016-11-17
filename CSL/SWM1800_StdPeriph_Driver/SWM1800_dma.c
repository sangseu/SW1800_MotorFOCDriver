/****************************************************************************************************************************************** 
* 文件名称: SWM1800_dma.c
* 功能说明:	SWM1800单片机的DMA功能驱动库
* 技术支持:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* 注意事项: 在不使能中断的情况下，也可以调用DMA_CH_INTStat()查询数据搬运是否完成，并调用DMA_CH_INTClr()清除完成标志
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
#include "SWM1800_dma.h"


/****************************************************************************************************************************************** 
* 函数名称: DMA_CH_Config()
* 功能说明:	DMA通道配置
* 输    入: uint32_t chn			指定要配置的通道，有效值有DMA_CHR_ADC、DMA_CHR_SDADC、DMA_CHR_CAN
*			uint32_t ram_addr		数据要被搬运到RAM中的地址，必须字对齐
*			uint32_t num_word		要搬运的数据字数，注意，单位是字，不是字节
*			uint32_t int_en			中断使能，1 数据搬运完成后产生中断    0 数据搬运完成后不产生中断
* 输    出: 无
* 注意事项: 在存储器间（如Flash和RAM间）搬运数据请使用DMA_CHM_Config()
******************************************************************************************************************************************/
void DMA_CH_Config(uint32_t chn, uint32_t ram_addr, uint32_t num_word, uint32_t int_en)
{
	DMA->EN = 1;			//每个通道都有自己独立的开关控制，所以总开关可以是一直开启的
	
	DMA_CH_Close(chn);		//配置前先关闭该通道
	
	switch(chn)
	{
	case DMA_CHR_ADC:
		ADC->CTRL |= (0x01 << ADC_CTRL_DMAEN_Pos) | (0x01 << ADC_CTRL_RES2FF_Pos);
		
		DMA->CH[chn].SRC = 0x5000D094;		//ADC->FFDATA寄存器
		break;
	
	case DMA_CHR_SDADC:
		SDADC->CTRL |= (0x01 << SDADC_CTRL_DMAEN_Pos);
		
		DMA->CH[chn].SRC = 0x50040044;		//SDADC->DATA寄存器
		break;
	
	case DMA_CHR_CAN:
		CAN->CR |= (0x01 << CAN_CR_DMAEN_Pos);
		
		DMA->CH[chn].SRC = 0x50020040;		//CAN->RXFRAME寄存器
		break;
	}
	
	DMA->CH[chn].DST = ram_addr;
	
	DMA->CH[chn].CR = ((num_word*4-1) << DMA_CR_LEN_Pos);
	
	DMA_CH_INTClr(chn);							//清除中断标志
	if(int_en)	DMA_CH_INTEn(chn);
	else		DMA_CH_INTDis(chn);
}

/****************************************************************************************************************************************** 
* 函数名称: DMA_CHM_Config()
* 功能说明:	DMA通道配置，用于存储器间（如Flash和RAM间）搬运数据
* 输    入: uint32_t chn			指定要配置的通道，有效值有DMA_CHW_FLASH、DMA_CHR_FLASH
*			uint32_t src_addr		对DMA_CHW_FLASH通道，源地址是RAM中地址，字对齐；    对DMA_CHR_FLASH通道，源地址是Flash中地址，字对齐
*			uint32_t dst_addr		对DMA_CHW_FLASH通道，目标直至是Flash中地址，字对齐；对DMA_CHR_FLASH通道，目标地址是RAM中地址，字对齐
*			uint32_t num_word		要搬运的数据字数，注意，单位是字，不是字节
*			uint32_t int_en			中断使能，1 数据搬运完成后产生中断    0 数据搬运完成后不产生中断
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void DMA_CHM_Config(uint32_t chn, uint32_t src_addr, uint32_t dst_addr, uint32_t num_word, uint32_t int_en)
{
	DMA->EN = 1;			//每个通道都有自己独立的开关控制，所以总开关可以是一直开启的
	
	DMA_CH_Close(chn);		//配置前先关闭该通道
	
	DMA->CH[chn].SRC = src_addr;
	DMA->CH[chn].DST = dst_addr;
	
	DMA->CH[chn].CR &= ~DMA_CR_LEN_Msk;
	DMA->CH[chn].CR |= ((num_word*4-1) << DMA_CR_LEN_Pos);
	
	DMA_CH_INTClr(chn);							//清除中断标志
	if(int_en)	DMA_CH_INTEn(chn);
	else		DMA_CH_INTDis(chn);
}

/****************************************************************************************************************************************** 
* 函数名称: DMA_CH_Open()
* 功能说明:	DMA通道打开
* 输    入: uint32_t chn			指定要配置的通道，有效值有DMA_CHW_FLASH、DMA_CHR_FLASH、DMA_CHR_ADC、DMA_CHR_SDADC、DMA_CHR_CAN
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void DMA_CH_Open(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_FLASH:
		DMA->CH[chn].CR |= (0x01 << DMA_CR_WEN_Pos);
		break;
	
	case DMA_CHR_FLASH:
	case DMA_CHR_ADC:		
	case DMA_CHR_SDADC:		
	case DMA_CHR_CAN:
		DMA->CH[chn].CR |= (0x01 << DMA_CR_REN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称: DMA_CH_Close()
* 功能说明:	DMA通道关闭
* 输    入: uint32_t chn			指定要配置的通道，有效值有DMA_CHW_FLASH、DMA_CHR_FLASH、DMA_CHR_ADC、DMA_CHR_SDADC、DMA_CHR_CAN
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void DMA_CH_Close(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_FLASH:
		DMA->CH[chn].CR &= ~(0x01 << DMA_CR_WEN_Pos);
		break;
	
	case DMA_CHR_FLASH:
	case DMA_CHR_ADC:		
	case DMA_CHR_SDADC:		
	case DMA_CHR_CAN:
		DMA->CH[chn].CR &= ~(0x01 << DMA_CR_REN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称: DMA_CH_INTEn()
* 功能说明:	DMA中断使能，数据搬运完成后触发中断
* 输    入: uint32_t chn			指定要配置的通道，有效值有DMA_CHW_FLASH、DMA_CHR_FLASH、DMA_CHR_ADC、DMA_CHR_SDADC、DMA_CHR_CAN
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void DMA_CH_INTEn(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_FLASH:
		DMA->IE |= (1 << DMA_IE_WFLASH_Pos);
		DMA->IM &= ~(1 << DMA_IM_WFLASH_Pos);
		break;
	
	case DMA_CHR_FLASH:
		DMA->IE |= (1 << DMA_IE_RFLASH_Pos);
		DMA->IM &= ~(1 << DMA_IM_RFLASH_Pos);
		break;
	
	case DMA_CHR_ADC:
		DMA->IE |= (1 << DMA_IE_ADC_Pos);
		DMA->IM &= ~(1 << DMA_IM_ADC_Pos);
		break;
	
	case DMA_CHR_SDADC:
		DMA->IE |= (1 << DMA_IE_SDADC_Pos);
		DMA->IM &= ~(1 << DMA_IM_SDADC_Pos);
		break;
	
	case DMA_CHR_CAN:
		DMA->IE |= (1 << DMA_IE_CAN_Pos);
		DMA->IM &= ~(1 << DMA_IM_CAN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称: DMA_CH_INTDis()
* 功能说明:	DMA中断禁止，数据搬运完成后不触发中断
* 输    入: uint32_t chn			指定要配置的通道，有效值有DMA_CHW_FLASH、DMA_CHR_FLASH、DMA_CHR_ADC、DMA_CHR_SDADC、DMA_CHR_CAN
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void DMA_CH_INTDis(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_FLASH:
		DMA->IM |= (1 << DMA_IM_WFLASH_Pos);
		DMA->IE &= ~(1 << DMA_IE_WFLASH_Pos);
		break;
	
	case DMA_CHR_FLASH:
		DMA->IM |= (1 << DMA_IM_RFLASH_Pos);
		DMA->IE &= ~(1 << DMA_IE_RFLASH_Pos);
		break;
	
	case DMA_CHR_ADC:
		DMA->IM |= (1 << DMA_IM_ADC_Pos);
		DMA->IE &= ~(1 << DMA_IE_ADC_Pos);
		break;
	
	case DMA_CHR_SDADC:
		DMA->IM |= (1 << DMA_IM_SDADC_Pos);
		DMA->IE &= ~(1 << DMA_IE_SDADC_Pos);
		break;
	
	case DMA_CHR_CAN:
		DMA->IM |= (1 << DMA_IM_CAN_Pos);
		DMA->IE &= ~(1 << DMA_IE_CAN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称: DMA_CH_INTClr()
* 功能说明:	DMA中断标志清除
* 输    入: uint32_t chn			指定要配置的通道，有效值有DMA_CHW_FLASH、DMA_CHR_FLASH、DMA_CHR_ADC、DMA_CHR_SDADC、DMA_CHR_CAN
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void DMA_CH_INTClr(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_FLASH:
		DMA->IF = (1 << DMA_IF_WFLASH_Pos);
		break;
	
	case DMA_CHR_FLASH:
		DMA->IF = (1 << DMA_IF_RFLASH_Pos);
		break;
	
	case DMA_CHR_ADC:
		DMA->IF = (1 << DMA_IF_ADC_Pos);
		break;
	
	case DMA_CHR_SDADC:
		DMA->IF = (1 << DMA_IF_SDADC_Pos);
		break;
	
	case DMA_CHR_CAN:
		DMA->IF = (1 << DMA_IF_CAN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称: DMA_CH_INTStat()
* 功能说明:	DMA中断状态查询
* 输    入: uint32_t chn			指定要配置的通道，有效值有DMA_CHW_FLASH、DMA_CHR_FLASH、DMA_CHR_ADC、DMA_CHR_SDADC、DMA_CHR_CAN
* 输    出: uint32_t				1 数据搬运完成    0 数据搬运未完成
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t DMA_CH_INTStat(uint32_t chn)
{
	uint32_t stat = 0;
	
	switch(chn)
	{
	case DMA_CHW_FLASH:
		stat = (DMA->IF & (1 << DMA_IF_WFLASH_Pos)) ? 1 : 0;
		break;
	
	case DMA_CHR_FLASH:
		stat = (DMA->IF & (1 << DMA_IF_RFLASH_Pos)) ? 1 : 0;
		break;
	
	case DMA_CHR_ADC:
		stat = (DMA->IF & (1 << DMA_IF_ADC_Pos)) ? 1 : 0;
		break;
	
	case DMA_CHR_SDADC:
		stat = (DMA->IF & (1 << DMA_IF_SDADC_Pos)) ? 1 : 0;
		break;
	
	case DMA_CHR_CAN:
		stat = (DMA->IF & (1 << DMA_IF_CAN_Pos)) ? 1 : 0;
		break;
	}
	
	return stat;
}
