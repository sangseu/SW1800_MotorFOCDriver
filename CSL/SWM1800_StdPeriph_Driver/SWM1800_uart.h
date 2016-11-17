#ifndef __SWM1800_UART_H__
#define __SWM1800_UART_H__

typedef struct {
	uint32_t Baudrate;
	
	uint8_t  RXThreshold;			//取值0--7
	uint8_t  RXThresholdIEn;		//当RX FIFO中数据个数 >= RXThreshold时触发中断
	
	uint8_t  TXThreshold;			//取值0--7
	uint8_t  TXThresholdIEn;		//当TX FIFO中数据个数 <= TXThreshold时触发中断
	
	uint8_t  TimeoutTime;			//超时时长 = TimeoutTime/(Baudrate/10) 秒
	uint8_t  TimeoutIEn;			//超时中断，超过 TimeoutTime/(Baudrate/10) 秒没有在RX线上接收到数据时触发中断
} UART_InitStructure;

void UART_Init(UART_TypeDef * UARTx, UART_InitStructure * initStruct);	//UART串口初始化
void UART_Open(UART_TypeDef * UARTx);
void UART_Close(UART_TypeDef * UARTx);

void UART_WriteByte(UART_TypeDef * UARTx, uint8_t data);				//发送一个字节数据
uint8_t UART_ReadByte(UART_TypeDef * UARTx);							//读取一个字节数据

uint32_t UART_IsDataValid(UART_TypeDef * UARTx);
uint32_t UART_IsTXBusy(UART_TypeDef * UARTx);
uint32_t UART_IsRXFIFOEmpty(UART_TypeDef * UARTx);						//接收FIFO是否空，如果不空则可以继续UART_ReadByte()
uint32_t UART_IsTXFIFOFull(UART_TypeDef * UARTx);						//发送FIFO是否满，如果不满则可以继续UART_WriteByte()


void UART_SetBaudrate(UART_TypeDef * UARTx, uint32_t baudrate);			//设置波特率
uint32_t UART_GetBaudrate(UART_TypeDef * UARTx);			 			//获取当前使用的波特率


void UART_INTRXThresholdEn(UART_TypeDef * UARTx);
void UART_INTRXThresholdDis(UART_TypeDef * UARTx);
uint32_t UART_INTRXThresholdStat(UART_TypeDef * UARTx);
void UART_INTTXThresholdEn(UART_TypeDef * UARTx);
void UART_INTTXThresholdDis(UART_TypeDef * UARTx);
uint32_t UART_INTTXThresholdStat(UART_TypeDef * UARTx);
void UART_INTTimeoutEn(UART_TypeDef * UARTx);
void UART_INTTimeoutDis(UART_TypeDef * UARTx);
uint32_t UART_INTTimeoutStat(UART_TypeDef * UARTx);


#endif //__SWM1800_UART_H__
