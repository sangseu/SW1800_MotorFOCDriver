#ifndef __SWM1500_UART_H__
#define __SWM1500_UART_H__

#include "SWM1500.h"

typedef struct {
	uint32_t baudrate;
	uint8_t data_len;		//0 5位数据位		1 6位数据位		2 7位数据位		3 8位数据位
	uint8_t stop_len;		//0 1位停止位		1 2位停止位
	uint8_t parity;			//0 无校验		1 奇校验		2 偶校验
	uint8_t rxfifo;			//0 不使用FIFO	1 深度为1字节	4 深度为4字节	8 深度为8字节	14 深度为14字节
	uint8_t RBR_IE;			//RBR寄存器有数据或RX FIFO中数据到达触发等级 中断使能
	uint8_t THR_IE;			//THR空或TX FIFO空 中断使能
	uint8_t LSR_IE;			//帧错误、校验错误、溢出、Break 指示 中断使能	
} UART_InitStructure;

void UART_Init(UART_T * UARTx,UART_InitStructure * initStruct);		//UART串口初始化，包括数据位长度、停止位长度、校验位、波特率、FIFO触发深度、中断使能等的设置
void UART_Open(UART_T * UARTx);										//UART串口使能，使能是指给UART模块提供工作时钟
void UART_Close(UART_T * UARTx);									//UART串口禁能，禁能是指切断UART模块的时钟供给，从而减小能耗	
void UART_WriteByte(UART_T * UARTx,uint8_t byt);					//发送一个字节数据
uint8_t UART_ReadByte(UART_T * UARTx);								//读取一个字节数据
void UART_WriteBytes(UART_T * UARTx,char buf[],uint32_t len);	//发送len个字节数据
uint32_t UART_ReadBytes(UART_T * UARTx,char buf[],uint32_t len);	//读取len个字节数据

void UART_SetBaudrate(UART_T * UARTx,uint32_t baudrate);			//设置波特率
uint32_t UART_GetBaudrate(UART_T * UARTx);			 				//获取当前使用的波特率
void UART_SetDataLen(UART_T * UARTx,uint32_t data_len);				//设置数据位长度
uint32_t UART_GetDataLen(UART_T * UARTx);							//获取当前的数据位长度
void UART_SetStopLen(UART_T * UARTx,uint32_t stop_len);				//设置停止位长度
uint32_t UART_GetStopLen(UART_T * UARTx);			   				//获取当前的停止位长度
void UART_SetParity(UART_T * UARTx,uint32_t parity);  				//设置校验位模式
uint32_t UART_GetParity(UART_T * UARTx);			  				//获取当前校验位模式
void UART_SetRXFIFOTL(UART_T * UARTx,uint32_t level); 				//设置RX FIFO Trigger Level（即RX FIFO接收到几个数据字节后触发RX FIFO满中断）
uint32_t UART_GetRXFIFOTL(UART_T * UARTx);		  					//获取串口当前的RX FIFO Trigger Level（即RX FIFO接收到几个数据字节后触发RX FIFO满中断）

void UART_IntRBREn(UART_T * UARTx);	 				//使能接收数据可用（即RBR中有数据）中断
void UART_IntRBRDis(UART_T * UARTx);		  		//禁能接收数据可用（即RBR中有数据）中断
void UART_IntTHREn(UART_T * UARTx);					//使能发送保持寄存器空中断
void UART_IntTHRDis(UART_T * UARTx);				//禁能发送保持寄存器空中断
void UART_IntLSREn(UART_T * UARTx);					//使能线状态（指帧错误、校验错误、溢出、Break 指示）中断
void UART_IntLSRDis(UART_T * UARTx);				//禁能线状态（指帧错误、校验错误、溢出、Break 指示）中断
uint32_t UART_IntCurrent(UART_T * UARTx); 			//获取当前的中断状态，确定当前发生的是哪种中断

uint32_t UART_IsDataAvailable(UART_T * UARTx);		//RBR中是否有有效字节
uint32_t UART_IsRXOverflow(UART_T * UARTx);	  		//RX是否发生溢出错误
uint32_t UART_IsParityError(UART_T * UARTx);  		//是否发生了校验错误
uint32_t UART_IsFrameError(UART_T * UARTx);	 		//是否发生了帧错误
uint32_t UART_HasBreakIndicate(UART_T * UARTx);		//是否发现Break Indicator
uint32_t UART_IsTHREmpty(UART_T * UARTx);	   		//THR（Transmit Hold Register，即发送保持寄存器）是否为空
uint32_t UART_IsTSREmpty(UART_T * UARTx);	   		//TSR（Transmit Shift Register，即发送移位寄存器）是否为空
uint32_t UART_RXFIFOError(UART_T * UARTx);		 	//RX FIFO中是否有错误（校验错误、帧错误、Break Indicator）


#endif //__SWM1500_UART_H__
