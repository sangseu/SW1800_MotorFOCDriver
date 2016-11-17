/****************************************************************************************************************************************** 
* 文件名称:	SWM1800_i2c.c
* 功能说明:	SWM1800单片机的I2C串行接口功能驱动库
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
#include "SWM1800_i2c.h"


/****************************************************************************************************************************************** 
* 函数名称:	I2C_Init()
* 功能说明:	I2C初始化
* 输    入: I2C_TypeDef * I2Cx		指定要被设置的I2C，有效值包括I2C0、I2C1
*			I2C_InitStructure * initStruct	包含I2C相关设定值的结构体
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void I2C_Init(I2C_TypeDef * I2Cx, I2C_InitStructure * initStruct)
{
	switch((uint32_t)I2Cx)
	{
	case ((uint32_t)I2C0):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_I2C0_Pos);
		break;
	
	case ((uint32_t)I2C1):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_I2C1_Pos);
		break;
	}
	
	I2C_Close(I2Cx);	//一些关键寄存器只能在I2C关闭时设置
	
	if(initStruct->Master == 1)
	{
		I2Cx->SLVCR &= ~I2C_SLVCR_SLAVE_Msk;
		I2Cx->SLVCR |= (0 << I2C_SLVCR_SLAVE_Pos);
		
		I2Cx->CLKDIV = SystemCoreClock/5/initStruct->MstClk - 1;
		
		I2Cx->MSTCMD = (I2Cx->MSTCMD & (~I2C_MSTCMD_IF_Msk)) | (1 << I2C_MSTCMD_IF_Pos);	//使能中断之前先清除中断标志
		I2Cx->CTRL &= ~I2C_CTRL_MSTIE_Msk;
		I2Cx->CTRL |= (initStruct->MstIEn << I2C_CTRL_MSTIE_Pos);
	}
	else
	{
		I2Cx->SLVCR &= ~I2C_SLVCR_SLAVE_Msk;
		I2Cx->SLVCR |= (1 << I2C_SLVCR_SLAVE_Pos);
		
		I2Cx->SLVCR &= ~(I2C_SLVCR_ADDR7b_Msk | I2C_SLVCR_ADDR_Msk);
		I2Cx->SLVCR |= (1 << I2C_SLVCR_ACK_Pos) |
					   (initStruct->Addr7b << I2C_SLVCR_ADDR7b_Pos) |
					   (initStruct->SlvAddr << I2C_SLVCR_ADDR_Pos);
		
		I2Cx->SLVIF = I2C_SLVIF_RXEND_Msk | I2C_SLVIF_TXEND_Msk | I2C_SLVIF_STADET_Msk | I2C_SLVIF_STODET_Msk;	//清中断标志
		I2Cx->SLVCR &= ~(I2C_SLVCR_IE_RXEND_Msk | I2C_SLVCR_IE_TXEND_Msk | I2C_SLVCR_IE_STADET_Msk | I2C_SLVCR_IE_STODET_Msk |
						 I2C_SLVCR_IE_RDREQ_Msk | I2C_SLVCR_IE_WRREQ_Msk);
		I2Cx->SLVCR |= (initStruct->SlvRxEndIEn << I2C_SLVCR_IE_RXEND_Pos) |
					   (initStruct->SlvTxEndIEn << I2C_SLVCR_IE_TXEND_Pos) |
					   (initStruct->SlvSTADetIEn << I2C_SLVCR_IE_STADET_Pos) |
					   (initStruct->SlvSTODetIEn << I2C_SLVCR_IE_STODET_Pos) |
					   (initStruct->SlvRdReqIEn << I2C_SLVCR_IE_RDREQ_Pos)  |
					   (initStruct->SlvWrReqIEn << I2C_SLVCR_IE_WRREQ_Pos);
	}
}

/****************************************************************************************************************************************** 
* 函数名称:	I2C_Open()
* 功能说明:	I2C打开，允许收发
* 输    入: I2C_TypeDef * I2Cx		指定要被设置的I2C，有效值包括I2C0、I2C1
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void I2C_Open(I2C_TypeDef * I2Cx)
{
	I2Cx->CTRL |= (0x01 << I2C_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	I2C_Close()
* 功能说明:	I2C关闭，禁止收发
* 输    入: I2C_TypeDef * I2Cx		指定要被设置的I2C，有效值包括I2C0、I2C1
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void I2C_Close(I2C_TypeDef * I2Cx)
{
	I2Cx->CTRL &= ~I2C_CTRL_EN_Msk;
}
