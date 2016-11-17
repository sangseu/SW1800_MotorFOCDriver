#ifndef __SWM1800_ADC_H__
#define	__SWM1800_ADC_H__

typedef struct {
	uint8_t clk_src;		//ADC转换时钟源：ADC_CLKSRC_HRC_DIV4、ADC_CLKSRC_XTAL_DIV8
	uint8_t channels;		//ADC转换通道选中，ADC_CH0、ADC_CH1、... ... 、ADC_CH7及其组合（即“按位或”运算）
	uint8_t trig_src;		//ADC触发方式：ADC_TRIGSRC_SW、ADC_TRIGSRC_PWM、ADC_TRIGSRC_TIMR2、ADC_TRIGSRC_TIMR3
	uint8_t Continue;		//在软件触发模式下：1 连续转换模式，启动后一直采样、转换，直到软件清除START位
							//                  0 单次转换模式，转换完成后START位自动清除停止转换
	uint8_t EOC_IEn;		//EOC中断使能，可针对每个通道设置，其有效值为ADC_CH0、ADC_CH1、... ... 、ADC_CH7及其组合（即“按位或”运算）
	uint8_t OVF_IEn;		//OVF中断使能，可针对每个通道设置，其有效值为ADC_CH0、ADC_CH1、... ... 、ADC_CH7及其组合（即“按位或”运算）
} ADC_InitStructure;

#define ADC_CH0		0x01
#define ADC_CH1		0x02
#define ADC_CH2		0x04
#define ADC_CH3		0x08
#define ADC_CH4		0x10
#define ADC_CH5		0x20
#define ADC_CH6		0x40
#define ADC_CH7		0x80

#define ADC_CLKSRC_HRC_DIV4		0	//内部高频RC振荡器4分频
#define ADC_CLKSRC_XTAL_DIV8	1	//外部晶体振荡器8分频

#define ADC_TRIGSRC_SW			0	//软件触发，即ADC->START.GO写1启动转换
#define ADC_TRIGSRC_PWM			1
#define ADC_TRIGSRC_TIMR2		2
#define ADC_TRIGSRC_TIMR3		3


void ADC_Init(ADC_TypeDef * ADCx, ADC_InitStructure * initStruct);		//ADC模数转换器初始化
void ADC_Open(ADC_TypeDef * ADCx);							//ADC开启，可以软件启动、或硬件触发ADC转换
void ADC_Close(ADC_TypeDef * ADCx);							//ADC关闭，无法软件启动、或硬件触发ADC转换
void ADC_Start(ADC_TypeDef * ADCx);							//启动指定ADC，开始模数转换
void ADC_Stop(ADC_TypeDef * ADCx);							//关闭指定ADC，停止模数转换

uint32_t ADC_Read(ADC_TypeDef * ADCx, uint32_t chn);		//从指定通道读取转换结果
uint32_t ADC_IsEOC(ADC_TypeDef * ADCx, uint32_t chn);		//指定通道是否End Of Conversion

void ADC_ChnOpen(ADC_TypeDef * ADCx, uint32_t chns);		//ADC通道开启，模数转换会在开启的通道上依次采样转换
void ADC_ChnClose(ADC_TypeDef * ADCx, uint32_t chns);		//ADC通道关闭


void ADC_IntEOCEn(ADC_TypeDef * ADCx, uint32_t chn);		//转换完成中断使能
void ADC_IntEOCDis(ADC_TypeDef * ADCx, uint32_t chn);		//转换完成中断禁止
void ADC_IntEOCClr(ADC_TypeDef * ADCx, uint32_t chn);		//转换完成中断标志清除
uint32_t ADC_IntEOCStat(ADC_TypeDef * ADCx, uint32_t chn);	//转换完成中断状态
void ADC_IntOVFEn(ADC_TypeDef * ADCx, uint32_t chn);		//数据溢出中断使能
void ADC_IntOVFDis(ADC_TypeDef * ADCx, uint32_t chn);		//数据溢出中断禁止
void ADC_IntOVFClr(ADC_TypeDef * ADCx, uint32_t chn);		//数据溢出中断标志清除
uint32_t ADC_IntOVFStat(ADC_TypeDef * ADCx, uint32_t chn);	//数据溢出中断状态



#endif //__SWM1800_ADC_H__
