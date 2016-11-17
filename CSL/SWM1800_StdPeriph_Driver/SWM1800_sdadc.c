/****************************************************************************************************************************************** 
* 文件名称: SWM1800_sdadc.c
* 功能说明:	SWM1800单片机的SDADC驱动库
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
#include "SWM1800_sdadc.h"

/****************************************************************************************************************************************** 
* 函数名称: SDADC_Init()
* 功能说明:	Sigma Delta ADC模数转换器初始化
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，有效值包括SDADC
*			SDADC_InitStructure * initStruct	 包含SDADC各相关定值的结构体
* 输    出: 无
* 注意事项: SDADC的时钟源选择位同ADC的时钟源选择位是同一位，设置时注意保持一致，不要冲突
******************************************************************************************************************************************/
void SDADC_Init(SDADC_TypeDef * SDADCx, SDADC_InitStructure * initStruct)
{
	switch((uint32_t)SDADCx)
	{
	case ((uint32_t)SDADC):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_SDADC_Pos);
		
		SYS->CLKSEL &= ~SYS_CLKSEL_ADC_Msk;
		SYS->CLKSEL |= (initStruct->clk_src << SYS_CLKSEL_ADC_Pos);
		break;
	}
	
	SDADCx->CTRL = (initStruct->channels << SDADC_CTRL_CH0SEL_Pos)  |
				   (0 << SDADC_CTRL_RST_Pos)                        |
				   (1 << SDADC_CTRL_BIAS_Pos)                       |
				   (initStruct->Continue << SDADC_CTRL_CONT_Pos)    |
				   (0 << SDADC_CTRL_FAST_Pos)                       |
				   (initStruct->out_cali << SDADC_CTRL_OUTCALI_Pos) |
				   (0 << SDADC_CTRL_LOWCLK_Pos)                     |
				   (initStruct->refp_sel << SDADC_CTRL_REFP_Pos)    |
				   (initStruct->trig_src << SDADC_CTRL_TRIG_Pos)    |
				   (0 << SDADC_CTRL_DMAEN_Pos);
	
	SDADCx->IF = 0x1F;			//清除中断标志
	SDADCx->IE = (initStruct->EOC_IEn << SDADC_IE_EOC_Pos) 	  |
				 (initStruct->OVF_IEn << SDADC_IE_FFOV_Pos)   |
				 (initStruct->HFULL_IEn << SDADC_IE_FFHF_Pos) |
				 (initStruct->FULL_IEn << SDADC_IE_FFF_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_Config_Set()
* 功能说明:	SDADC支持3个配置，9个输入通道可选择其中一个作为自己的配置，此函数用于设置配置的具体内容
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
*			uint32_t CFGx				要设置哪个配置，可取值SDADC_CFG_A、SDADC_CFG_B、SDADC_CFG_C
*			uint32_t gian				增益，可取值SDADC_CFG_GAIN_1、SDADC_CFG_GAIN_2、... ...、SDADC_CFG_GAIN_1DIV2
*			uint32_t sigle_end			0 差分模式    1 单端模式
*			uint32_t refm_as_inn		0 通道的N输入脚作为N输入端    1 公共的AINREFM引脚作为N输入端
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_Config_Set(SDADC_TypeDef * SDADCx, uint32_t CFGx, uint32_t gian, uint32_t sigle_end, uint32_t refm_as_inn)
{
	switch(CFGx)
	{
	case SDADC_CFG_A:
		SDADCx->CFGA = (gian << SDADC_CFG_GAIN_Pos)    |
					   (sigle_end << SDADC_CFG_SE_Pos) |
					   (refm_as_inn << SDADC_CFG_REFM_Pos);
		break;
	
	case SDADC_CFG_B:
		SDADCx->CFGB = (gian << SDADC_CFG_GAIN_Pos)    |
					   (sigle_end << SDADC_CFG_SE_Pos) |
					   (refm_as_inn << SDADC_CFG_REFM_Pos);
		break;
	
	case SDADC_CFG_C:
		SDADCx->CFGB = (gian << SDADC_CFG_GAIN_Pos)    |
					   (sigle_end << SDADC_CFG_SE_Pos) |
					   (refm_as_inn << SDADC_CFG_REFM_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_Config_Cali()
* 功能说明:	SDADC支持3个配置，9个输入通道可选择其中一个作为自己的配置，此函数用于设置配置的具体内容
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
*			uint32_t CFGx				要设置哪个配置，可取值SDADC_CFG_A、SDADC_CFG_B、SDADC_CFG_C
*			uint32_t cali_com			校准公共端，可取值有SDADC_CALI_COM_GND、SDADC_CALI_COM_VDD_1DIV2、SDADC_CALI_COM_VDD
*			uint32_t cali_fast			快速校准，可将校准时间缩短至1/3
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_Config_Cali(SDADC_TypeDef * SDADCx, uint32_t CFGx, uint32_t cali_com, uint32_t cali_fast)
{
	uint32_t sdadc_CTRL = SDADCx->CTRL;		//保存SDADCx->CTRL寄存器的值
	
	SDADCx->CTRL = ((1 << 9) << SDADC_CTRL_CH0SEL_Pos)  		|	//选中通道9
				   (0 << SDADC_CTRL_RST_Pos)                   	|
				   (1 << SDADC_CTRL_EN_Pos)						|
				   (1 << SDADC_CTRL_BIAS_Pos)                  	|
				   (0 << SDADC_CTRL_CONT_Pos)    				|
				   (cali_fast << SDADC_CTRL_FAST_Pos)           |
				   (SDADC_OUT_RAW << SDADC_CTRL_OUTCALI_Pos) 	|
				   (0 << SDADC_CTRL_LOWCLK_Pos)                	|
				   (sdadc_CTRL & SDADC_CTRL_REFP_Msk)    		|
				   (cali_com << SDADC_CTRL_CALIN_Pos)			|
				   (SDADC_TRIGSRC_SW << SDADC_CTRL_TRIG_Pos)   	|
				   (0 << SDADC_CTRL_DMAEN_Pos);
	
	SDADCx->CFGS &= ~(3 << (9*2));			//选择要校准哪个配置
	SDADCx->CFGS |= (CFGx << (9*2));
	
	SDADC_Start(SDADC);						//开启转换，执行校准
	while((SDADC->STAT & SDADC_STAT_CALEOC_Msk) == 0);
	SDADC->STAT = 1 << SDADC_STAT_CALEOC_Pos;
	
	SDADCx->CTRL = sdadc_CTRL;				//恢复SDADCx->CTRL寄存器的值
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_Config_Sel()
* 功能说明:	SDADC支持3个配置，9个输入通道可选择其中一个作为自己的配置，此函数用于为通道选择配置
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
*			uint32_t CFGx				要设置哪个配置，可取值SDADC_CFG_A、SDADC_CFG_B、SDADC_CFG_C
*			uint32_t channels			选择使用配置CFGx的通道，可取值SDADC_CH0、SDADC_CH1、... ... 、SDADC_CH5及其组合（即“按位或”运算）
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_Config_Sel(SDADC_TypeDef * SDADCx, uint32_t CFGx, uint32_t channels)
{
	uint32_t i;
	
	for(i = 0; i < 10; i++)
	{
		if(channels & (1 << i))
		{
			SDADCx->CFGS &= ~(3 << (i*2));
			SDADCx->CFGS |= (CFGx << (i*2));
		}
	}
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_Open()
* 功能说明:	SDADC开启，可以软件启动、或硬件触发SDADC转换
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_Open(SDADC_TypeDef * SDADCx)
{
	SDADCx->CTRL |= (0x01 << SDADC_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_Close()
* 功能说明:	SDADC关闭，无法软件启动、或硬件触发SDADC转换
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_Close(SDADC_TypeDef * SDADCx)
{
	SDADCx->CTRL &= ~(0x01 << SDADC_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_Start()
* 功能说明:	软件触发模式下启动SDADC转换
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_Start(SDADC_TypeDef * SDADCx)
{
	SDADCx->START = (1 << SDADC_START_GO_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_Stop()
* 功能说明:	软件触发模式下停止SDADC转换
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_Stop(SDADC_TypeDef * SDADCx)
{
	SDADCx->START &= ~(0x01 << SDADC_START_GO_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_Read()
* 功能说明:	读取转换结果
* 输    入: SDADC_TypeDef * SDADCx		指定要读取的SDADC，可取值包括SDADC
* 输    出: int16_t						读取到的转换结果
*			uint32_t *chn				转换结果来自哪个通道，0 SDADC_CH0    1 SDADC_CH1、... ...、5 SDADC_CH5
* 注意事项: 无
******************************************************************************************************************************************/
int16_t SDADC_Read(SDADC_TypeDef * SDADCx, uint32_t *chn)
{
	uint32_t data = SDADCx->DATA;
	*chn = (data & SDADC_DATA_CHNUM_Msk) >> SDADC_DATA_CHNUM_Pos;
	
	return (data & SDADC_DATA_VALUE_Msk);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_IsEOC()
* 功能说明:	是否End Of Conversion
* 输    入: SDADC_TypeDef * SDADCx		指定要查询的SDADC，可取值包括SDADC
* 输    出: uint32_t					1 有通道完成转换    0 没有通道完成转换
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t SDADC_IsEOC(SDADC_TypeDef * SDADCx)
{
	return (SDADCx->STAT & SDADC_STAT_EOC_Msk) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_ChnOpen()
* 功能说明:	SDADC通道开启，模数转换会在开启的通道上依次采样转换
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
*			uint32_t chns				要打开的通道，有效值SDADC_CH0、SDADC_CH1、... ... 、SDADC_CH5及其组合（即“按位或”运算）
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_ChnOpen(SDADC_TypeDef * SDADCx, uint32_t chns)
{
	SDADCx->CTRL |= chns;
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_ChnClose()
* 功能说明:	SDADC通道关闭
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
*			uint32_t chns				要关闭的通道，有效值SDADC_CH0、SDADC_CH1、... ... 、SDADC_CH5及其组合（即“按位或”运算）	
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_ChnClose(SDADC_TypeDef * SDADCx, uint32_t chns)
{
	SDADCx->CTRL &= ~chns;
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_IntEOCEn()
* 功能说明:	转换完成中断使能
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_IntEOCEn(SDADC_TypeDef * SDADCx)
{
	SDADCx->IE |= (1 << SDADC_IE_EOC_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_IntEOCDis()
* 功能说明:	转换完成中断禁止
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_IntEOCDis(SDADC_TypeDef * SDADCx)
{
	SDADCx->IE &= ~(1 << SDADC_IE_EOC_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_IntEOCClr()
* 功能说明:	转换完成中断标志清除
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_IntEOCClr(SDADC_TypeDef * SDADCx)
{
	SDADCx->IF = (1 << SDADC_IF_EOC_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_IntEOCStat()
* 功能说明:	转换完成中断状态
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: uint32_t					1 通道完成了转换    0 通道未完成转换
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t SDADC_IntEOCStat(SDADC_TypeDef * SDADCx)
{	
	return (SDADCx->IF & SDADC_IF_EOC_Msk) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_IntOVFEn()
* 功能说明:	数据溢出中断使能
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_IntOVFEn(SDADC_TypeDef * SDADCx)
{
	SDADCx->IE |= (1 << SDADC_IE_FFOV_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_IntOVFDis()
* 功能说明:	数据溢出中断禁止
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_IntOVFDis(SDADC_TypeDef * SDADCx)
{	
	SDADCx->IE &= ~(1 << SDADC_IE_FFOV_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_IntOVFClr()
* 功能说明:	数据溢出中断标志清除
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void SDADC_IntOVFClr(SDADC_TypeDef * SDADCx)
{
	SDADCx->IF = (1 << SDADC_IF_FFOV_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	SDADC_IntOVFStat()
* 功能说明:	数据溢出中断状态
* 输    入: SDADC_TypeDef * SDADCx		指定要被设置的SDADC，可取值包括SDADC
* 输    出: uint32_t					1 有通道溢出    0 没有通道溢出
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t SDADC_IntOVFStat(SDADC_TypeDef * SDADCx)
{	
	return (SDADCx->IF & SDADC_IF_FFOV_Msk) ? 1 : 0;
}
