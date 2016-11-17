#ifndef __SWM1800_CAN_H__
#define __SWM1800_CAN_H__

#define CAN_FRAME_STD	0
#define CAN_FRAME_EXT	1

typedef struct {
	uint8_t  Mode;			//CAN_MODE_NORMAL、CAN_MODE_LISTEN、CAN_MODE_SELFTEST
	uint8_t  CAN_BS1;		//CAN_BS1_1tq、CAN_BS1_2tq、... ... 、CAN_BS1_16tq
	uint8_t  CAN_BS2;		//CAN_BS2_1tq、CAN_BS2_2tq、... ... 、CAN_BS2_8tq
	uint8_t  CAN_SJW;		//CAN_SWJ_1tq、CAN_SWJ_2tq、CAN_SWJ_3tq、CAN_SWJ_4tq
	uint32_t Baudrate;		//波特率，即位传输速率，取值1--1000000
	uint8_t  FilterMode;	//CAN_FILTER_16b、CAN_FILTER_32b
	union {
		uint32_t FilterMask32b;		//FilterCheck & (~FilterMask) == ID & (~FilterMask)的Message通过过滤
		struct {
			uint16_t FilterMask16bL;
			uint16_t FilterMask16bH;
		};
	};
	union {
		uint32_t FilterCheck32b;
		struct {
			uint16_t FilterCheck16bL;
			uint16_t FilterCheck16bH;
		};
	};
	uint8_t  RXNotEmptyIEn;		//接收FIFO非空，有数据可读
	uint8_t  TXNotFullIEn;		//发送FIFO非满，可以写数据发送
	uint8_t  ErrorWarnIEn;		//接收/发送错误计数器达到 Warning Limit
	uint8_t  ArbitLostIEn;		//CAN控制器丢失仲裁变成接收
} CAN_InitStructure;

#define CAN_MODE_NORMAL		0	//常规模式
#define CAN_MODE_LISTEN		1	//监听模式
#define CAN_MODE_SELFTEST	2	//自测模式

#define CAN_BS1_1tq			0
#define CAN_BS1_2tq			1
#define CAN_BS1_3tq			2
#define CAN_BS1_4tq			3
#define CAN_BS1_5tq			4
#define CAN_BS1_6tq			5
#define CAN_BS1_7tq			6
#define CAN_BS1_8tq			7
#define CAN_BS1_9tq			8
#define CAN_BS1_10tq		9
#define CAN_BS1_11tq		10
#define CAN_BS1_12tq		11
#define CAN_BS1_13tq		12
#define CAN_BS1_14tq		13
#define CAN_BS1_15tq		14
#define CAN_BS1_16tq		15

#define CAN_BS2_1tq			0
#define CAN_BS2_2tq			1
#define CAN_BS2_3tq			2
#define CAN_BS2_4tq			3
#define CAN_BS2_5tq			4
#define CAN_BS2_6tq			5
#define CAN_BS2_7tq			6
#define CAN_BS2_8tq			7

#define CAN_SWJ_1tq			0
#define CAN_SWJ_2tq			1
#define CAN_SWJ_3tq			2
#define CAN_SWJ_4tq			3

#define CAN_FILTER_16b		0	//两个16位过滤器
#define CAN_FILTER_32b		1	//一个32位过滤器


void CAN_Init(CAN_TypeDef * CANx, CAN_InitStructure * initStruct);
void CAN_Open(CAN_TypeDef * CANx);
void CAN_Close(CAN_TypeDef * CANx);

void CAN_Transmit(CAN_TypeDef * CANx, uint32_t format, uint32_t id, uint8_t data[], uint32_t size);

#endif //__SWM1800_CAN_H__
