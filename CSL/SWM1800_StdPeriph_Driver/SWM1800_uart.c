/****************************************************************************************************************************************** 
* 文件名称:	SWM1800_uart.c
* 功能说明:	SWM1800单片机的UART串口功能驱动库
* 技术支持:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* 注意事项: 没有编写LIN功能相关的函数
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
#include "SWM1800_uart.h"


/****************************************************************************************************************************************** 
* 函数名称:	UART_Init()
* 功能说明:	UART串口初始化
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
*			UART_InitStructure * initStruct    包含UART串口相关设定值的结构体
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_Init(UART_TypeDef * UARTx, UART_InitStructure * initStruct)
{	
	switch((uint32_t)UARTx)
	{
	case ((uint32_t)UART0):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_UART0_Pos);
		break;
	
	case ((uint32_t)UART1):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_UART1_Pos);
		break;
	
	case ((uint32_t)UART2):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_UART2_Pos);
		break;
	
	case ((uint32_t)UART3):
		SYS->CLKEN |= (0x01 << SYS_CLKEN_UART3_Pos);
		break;
	}
	
	UART_Close(UARTx);	//一些关键寄存器只能在串口关闭时设置
	
	UARTx->CTRL |= (0x01 << UART_CTRL_BAUDEN_Pos);
	UARTx->BAUD &= ~UART_BAUD_BAUD_Msk;
	UARTx->BAUD |= ((SystemCoreClock/16/initStruct->Baudrate - 1) << UART_BAUD_BAUD_Pos);
	
	UARTx->FIFO &= ~(UART_FIFO_RXTHR_Msk | UART_FIFO_TXTHR_Msk);
	UARTx->FIFO |= (initStruct->RXThreshold << UART_FIFO_RXTHR_Pos) | 
				   (initStruct->TXThreshold << UART_FIFO_TXTHR_Pos);
	
	UARTx->CTRL &= ~UART_CTRL_TOTIME_Msk;
	UARTx->CTRL |= (initStruct->TimeoutTime << UART_CTRL_TOTIME_Pos);
	
	UARTx->CTRL &= ~(UART_CTRL_RXIE_Msk | UART_CTRL_TXIE_Msk | UART_CTRL_TOIE_Msk);
	UARTx->CTRL |= (initStruct->RXThresholdIEn << UART_CTRL_RXIE_Pos) |
				   (initStruct->TXThresholdIEn << UART_CTRL_TXIE_Pos) |
				   (initStruct->TimeoutIEn << UART_CTRL_TOIE_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_Open()
* 功能说明:	UART串口打开
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_Open(UART_TypeDef * UARTx)
{
	UARTx->CTRL |= (0x01 << UART_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_Close()
* 功能说明:	UART串口关闭
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_Close(UART_TypeDef * UARTx)
{
	UARTx->CTRL &= ~(0x01 << UART_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_WriteByte()
* 功能说明:	发送一个字节数据
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，可取值包括UART0、UART1、UART2、UART3
*			uint8_t data			要发送的字节			
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_WriteByte(UART_TypeDef * UARTx, uint8_t data)
{
	UARTx->DATA = data;
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_ReadByte()
* 功能说明:	读取一个字节数据
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，可取值包括UART0、UART1、UART2、UART3
* 输    出: uint8_t					接收到的数据
* 注意事项: 读取数据之前需要先调用UART_IsDataValid()检测接收到的数据是否有错误（如帧错误）
******************************************************************************************************************************************/
uint8_t UART_ReadByte(UART_TypeDef * UARTx)
{
	return (UARTx->DATA & UART_DATA_DATA_Msk);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_IsDataValid()
* 功能说明:	接收到的数据是否完整有效，即是否有错误（如帧错误）
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: uint32_t				1 数据完整有效，没有错误    0 数据有错误
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t UART_IsDataValid(UART_TypeDef * UARTx)
{
	return (UARTx->DATA & UART_DATA_VALID_Msk) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_IsTXBusy()
* 功能说明:	UART是否正在发送数据
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: uint32_t				1 UART正在发送数据    0 数据已发完
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t UART_IsTXBusy(UART_TypeDef * UARTx)
{
	return (UARTx->CTRL & UART_CTRL_TXIDLE_Msk) ? 0 : 1;
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_IsRXFIFOEmpty()
* 功能说明:	接收FIFO是否为空，如果不空则说明其中有数据可以读取
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: uint32_t				1 接收FIFO空    0 接收FIFO非空
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t UART_IsRXFIFOEmpty(UART_TypeDef * UARTx)
{
	return (UARTx->CTRL & UART_CTRL_RXNE_Msk) ? 0 : 1;
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_IsTXFIFOFull()
* 功能说明:	发送FIFO是否为满，如果不满则可以继续向其中写入数据
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: uint32_t				1 发送FIFO满    0 发送FIFO不满
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t UART_IsTXFIFOFull(UART_TypeDef * UARTx)
{
	return (UARTx->CTRL & UART_CTRL_TXF_Msk) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_SetBaudrate()
* 功能说明:	设置波特率
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
*			uint32_t baudrate		要设置的波特率
* 输    出: 无
* 注意事项: 不要在串口工作时更改波特率，使用此函数前请先调用UART_Close()关闭串口
******************************************************************************************************************************************/
void UART_SetBaudrate(UART_TypeDef * UARTx, uint32_t baudrate)
{
	UARTx->BAUD &= ~UART_BAUD_BAUD_Msk;
	UARTx->BAUD |= ((SystemCoreClock/16/baudrate) << UART_BAUD_BAUD_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_GetBaudrate()
* 功能说明:	查询波特率
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: uint32_t				当前波特率
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t UART_GetBaudrate(UART_TypeDef * UARTx)
{
	return (UARTx->BAUD & UART_BAUD_BAUD_Msk);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_INTRXThresholdEn()
* 功能说明:	当RX FIFO中数据个数 >= RXThreshold时 触发中断
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_INTRXThresholdEn(UART_TypeDef * UARTx)
{
	UARTx->CTRL |= (0x01 << UART_CTRL_RXIE_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_INTRXThresholdDis()
* 功能说明:	当RX FIFO中数据个数 >= RXThreshold时 不触发中断
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_INTRXThresholdDis(UART_TypeDef * UARTx)
{
	UARTx->CTRL &= ~(0x01 << UART_CTRL_RXIE_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_INTRXThresholdStat()
* 功能说明:	是否RX FIFO中数据个数 >= RXThreshold
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: uint32_t				1 RX FIFO中数据个数 >= RXThreshold		0 RX FIFO中数据个数 < RXThreshold
* 注意事项: RXIF = RXTHRF & RXIE
******************************************************************************************************************************************/
uint32_t UART_INTRXThresholdStat(UART_TypeDef * UARTx)
{
	return (UARTx->BAUD & UART_BAUD_RXIF_Msk) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_INTTXThresholdEn()
* 功能说明:	当TX FIFO中数据个数 <= TXThreshold时 触发中断
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_INTTXThresholdEn(UART_TypeDef * UARTx)
{
	UARTx->CTRL |= (0x01 << UART_CTRL_TXIE_Pos);	
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_INTTXThresholdDis()
* 功能说明:	当TX FIFO中数据个数 <= TXThreshold时 不触发中断
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_INTTXThresholdDis(UART_TypeDef * UARTx)
{
	UARTx->CTRL &= ~(0x01 << UART_CTRL_TXIE_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_INTTXThresholdStat()
* 功能说明:	是否TX FIFO中数据个数 <= TXThreshold
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: uint32_t				1 TX FIFO中数据个数 <= TXThreshold		0 TX FIFO中数据个数 > TXThreshold
* 注意事项: TXIF = TXTHRF & TXIE
******************************************************************************************************************************************/
uint32_t UART_INTTXThresholdStat(UART_TypeDef * UARTx)
{
	return (UARTx->BAUD & UART_BAUD_TXIF_Msk) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_INTTimeoutEn()
* 功能说明:	接收发生超时时 触发中断
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_INTTimeoutEn(UART_TypeDef * UARTx)
{
	UARTx->CTRL |= (0x01 << UART_CTRL_TOIE_Pos);	
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_INTTimeoutDis()
* 功能说明:	接收发生超时时 不触发中断
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void UART_INTTimeoutDis(UART_TypeDef * UARTx)
{
	UARTx->CTRL &= ~(0x01 << UART_CTRL_TOIE_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	UART_INTTimeoutStat()
* 功能说明:	是否发生了接收超时，即超过 TimeoutTime/(Baudrate/10) 秒没有在RX线上接收到数据时触发中断
* 输    入: UART_TypeDef * UARTx	指定要被设置的UART串口，有效值包括UART0、UART1、UART2、UART3
* 输    出: uint32_t				1 发生了接收超时		0 未发生接收超时
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t UART_INTTimeoutStat(UART_TypeDef * UARTx)
{
	return (UARTx->BAUD & UART_BAUD_TOIF_Msk) ? 1 : 0;
}

