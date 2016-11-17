/****************************************************************************************************************************************** 
* 文件名称:	SWM1800_adc.c
* 功能说明:	SWM1800单片机的ADC数模转换器功能驱动库
* 技术支持:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* 注意事项:
* 版本日期: V1.0.0		2016年1月30日
* 升级记录: 
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
#include "SWM1800_adc.h"


/****************************************************************************************************************************************** 
* 函数名称: ADC_Init()
* 功能说明:	ADC模数转换器初始化
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，有效值包括ADC
*			ADC_InitStructure * initStruct		包含ADC各相关定值的结构体
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_Init(ADC_TypeDef * ADCx, ADC_InitStructure * initStruct)
{
	switch((uint32_t)ADCx)
	{
	case ((uint32_t)ADC):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_ADC_Pos);
		
		SYS->CLKSEL &= ~SYS_CLKSEL_ADC_Msk;
		SYS->CLKSEL |= (initStruct->clk_src << SYS_CLKSEL_ADC_Pos);
		break;
	}
	
	ADC_Close(ADCx);		//一些关键寄存器只能在ADC关闭时设置
	
	ADCx->CTRL &= ~(ADC_CTRL_CH0_Msk | ADC_CTRL_CH1_Msk | ADC_CTRL_CH2_Msk | ADC_CTRL_CH3_Msk |
					ADC_CTRL_CH4_Msk | ADC_CTRL_CH5_Msk | ADC_CTRL_CH6_Msk | ADC_CTRL_CH7_Msk);
	ADCx->CTRL |= (initStruct->channels << ADC_CTRL_CH0_Pos);
	
	ADCx->CTRL &= ~(ADC_CTRL_TRIG_Msk | ADC_CTRL_CONT_Msk);
	ADCx->CTRL |= (initStruct->trig_src << ADC_CTRL_TRIG_Pos) |
				  (initStruct->Continue << ADC_CTRL_CONT_Pos);
	
	ADCx->IF = 0xFFFF;	//清除中断标志
	
	ADCx->IE &= ~(ADC_IE_CH0EOC_Msk | ADC_IE_CH1EOC_Msk | ADC_IE_CH2EOC_Msk | ADC_IE_CH3EOC_Msk |
				  ADC_IE_CH4EOC_Msk | ADC_IE_CH5EOC_Msk | ADC_IE_CH6EOC_Msk | ADC_IE_CH7EOC_Msk);
	ADCx->IE |= (((initStruct->EOC_IEn & ADC_CH0) ? 1 : 0) << ADC_IE_CH0EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH1) ? 1 : 0) << ADC_IE_CH1EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH2) ? 1 : 0) << ADC_IE_CH2EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH3) ? 1 : 0) << ADC_IE_CH3EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH4) ? 1 : 0) << ADC_IE_CH4EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH5) ? 1 : 0) << ADC_IE_CH5EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH6) ? 1 : 0) << ADC_IE_CH6EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH7) ? 1 : 0) << ADC_IE_CH7EOC_Pos);
				
	ADCx->IE &= ~(ADC_IE_CH0OVF_Msk | ADC_IE_CH1OVF_Msk | ADC_IE_CH2OVF_Msk | ADC_IE_CH3OVF_Msk |
				  ADC_IE_CH4OVF_Msk | ADC_IE_CH5OVF_Msk | ADC_IE_CH6OVF_Msk | ADC_IE_CH7OVF_Msk);
	ADCx->IE |= (((initStruct->OVF_IEn & ADC_CH0) ? 1 : 0) << ADC_IE_CH0OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH1) ? 1 : 0) << ADC_IE_CH1OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH2) ? 1 : 0) << ADC_IE_CH2OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH3) ? 1 : 0) << ADC_IE_CH3OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH4) ? 1 : 0) << ADC_IE_CH4OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH5) ? 1 : 0) << ADC_IE_CH5OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH6) ? 1 : 0) << ADC_IE_CH6OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH7) ? 1 : 0) << ADC_IE_CH7OVF_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_Open()
* 功能说明:	ADC开启，可以软件启动、或硬件触发ADC转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_Open(ADC_TypeDef * ADCx)
{
	ADCx->CTRL |= (0x01 << ADC_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_Close()
* 功能说明:	ADC关闭，无法软件启动、或硬件触发ADC转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_Close(ADC_TypeDef * ADCx)
{
	ADCx->CTRL &= ~(0x01 << ADC_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_Start()
* 功能说明:	软件触发模式下启动ADC转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_Start(ADC_TypeDef * ADCx)
{
	ADCx->START = (0x01 << ADC_START_GO_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_Stop()
* 功能说明:	软件触发模式下停止ADC转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_Stop(ADC_TypeDef * ADCx)
{									 
	ADCx->START &= ~(0x01 << ADC_START_GO_Pos);
}

static uint32_t chn2idx(uint32_t chn)
{
	uint32_t idx = 0;
	
	switch(chn)
	{
		case 0x01: idx = 0; break;
		case 0x02: idx = 1; break;
		case 0x04: idx = 2; break;
		case 0x08: idx = 3; break;
		case 0x10: idx = 4; break;
		case 0x20: idx = 5; break;
		case 0x40: idx = 6; break;
		case 0x80: idx = 7; break;
	}
	
	return idx;
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_Read()
* 功能说明:	从指定通道读取转换结果
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要读取转换结果的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: uint32_t				读取到的转换结果
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t ADC_Read(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->CH[idx].DATA & ADC_DATA_VALUE_Msk);
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IsEOC()
* 功能说明:	指定通道是否End Of Conversion
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要查询状态的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: uint32_t				1 该通道完成了转换    0 该通道未完成转换
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t ADC_IsEOC(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->CH[idx].STAT & ADC_STAT_EOC_Msk) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_ChnOpen()
* 功能说明:	ADC通道开启，模数转换会在开启的通道上依次采样转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chns			要打开的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7及其组合（即“按位或”运算）
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_ChnOpen(ADC_TypeDef * ADCx, uint32_t chns)
{
	ADCx->CTRL |= chns;
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_ChnClose()
* 功能说明:	ADC通道关闭
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chns			要关闭的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7及其组合（即“按位或”运算）	
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_ChnClose(ADC_TypeDef * ADCx, uint32_t chns)
{
	ADCx->CTRL &= ~chns;
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntEOCEn()
* 功能说明:	转换完成中断使能
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntEOCEn(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE |= (0x01 << (idx*2));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntEOCDis()
* 功能说明:	转换完成中断禁止
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntEOCDis(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE &= ~(0x01 << (idx*2));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntEOCClr()
* 功能说明:	转换完成中断标志清除
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntEOCClr(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IF = (0x01 << (idx*2));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntEOCStat()
* 功能说明:	转换完成中断状态
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要查询的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: uint32_t				1 该通道完成了转换    0 该通道未完成转换
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t ADC_IntEOCStat(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->IF & (0x01 << (idx*2))) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntOVFEn()
* 功能说明:	数据溢出中断使能
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntOVFEn(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE |= (0x01 << (idx*2+1));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntOVFDis()
* 功能说明:	数据溢出中断禁止
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntOVFDis(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE &= ~(0x01 << (idx*2+1));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntOVFClr()
* 功能说明:	数据溢出中断标志清除
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntOVFClr(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IF = (0x01 << (idx*2+1));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntOVFStat()
* 功能说明:	数据溢出中断状态
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要查询的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: uint32_t				1 有通道溢出    0 没有通道溢出
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t ADC_IntOVFStat(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->IF & (0x01 << (idx*2+1))) ? 1 : 0;
}
