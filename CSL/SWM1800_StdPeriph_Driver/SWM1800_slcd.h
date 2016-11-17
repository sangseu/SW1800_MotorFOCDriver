#ifndef __SWM1800_SLCD_H__
#define __SWM1800_SLCD_H__

typedef struct {
	uint8_t Duty;		//占空比：SLCD_DUTY_1DIV4、SLCD_DUTY_1DIV3
	uint8_t Bias;		//偏置模式：SLCD_BIAS_1DIV3、SLCD_BIAX_1DIV2
	uint8_t FrameFreq;	//帧频率：SLCD_FRAMEFREQ_32Hz、SLCD_FRAMEFREQ_16Hz、SLCD_FRAMEFREQ_8Hz、SLCD_FRAMEFREQ_4Hz
	uint8_t DriveCurr;	//驱动电流：SLCD_DRIVECURR_8uA、SLCD_DRIVECURR_25uA、SLCD_DRIVECURR_50uA、SLCD_DRIVECURR_100uA
} SLCD_InitStructure;

#define SLCD_DUTY_1DIV4		0	// 1/4 Duty
#define SLCD_DUTY_1DIV3		1	// 1/3 Duty

#define SLCD_BIAS_1DIV3		0	// 1/3 Bias
#define SLCD_BIAX_1DIV2		1	// 1/2 Bias

#define SLCD_FRAMEFREQ_32Hz		0
#define SLCD_FRAMEFREQ_16Hz		1
#define SLCD_FRAMEFREQ_8Hz		2
#define SLCD_FRAMEFREQ_4Hz		3

#define SLCD_DRIVECURR_8uA		0
#define SLCD_DRIVECURR_25uA		1
#define SLCD_DRIVECURR_50uA		2
#define SLCD_DRIVECURR_100uA	3


void SLCD_Init(SLCD_TypeDef * SLCDx, SLCD_InitStructure * initStruct);
void SLCD_Open(SLCD_TypeDef * SLCDx);
void SLCD_Close(SLCD_TypeDef * SLCDx);

void SLCD_Clear(SLCD_TypeDef * SLCDx);
void SLCD_AllOn(SLCD_TypeDef * SLCDx);



/****************************************************************************************************************************************** 
* 函数名称:	SLCD_SegWrite()
* 功能说明:	将断码屏上[com, seg]指定的字段点亮（val=1）或熄灭（val=0）
* 输    入: uint32_t com		断码屏COM端，取值SLCD_COM0、SLCD_COM1、SLCD_COM2、SLCD_COM3
*			uint32_t seg		断码屏SEG端，取值SLCD_SEG0、SLCD_SEG1、... ... 、SLCD_SEG30、SLCD_SEG31
*			uint32_t val		1 点亮     0 熄灭
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
static __INLINE void SLCD_SegWrite(uint32_t com, uint32_t seg, uint32_t val)
{
	if(val == 0)
		SLCD->DATA[com] &= ~(1 << seg);
	else
		SLCD->DATA[com] |= (1 << seg);
}

#define SLCD_COM0	0
#define SLCD_COM1	1
#define SLCD_COM2	2
#define SLCD_COM3	3

#define SLCD_SEG0	0
#define SLCD_SEG1	1
#define SLCD_SEG2	2
#define SLCD_SEG3	3
#define SLCD_SEG4	4
#define SLCD_SEG5	5
#define SLCD_SEG6	6
#define SLCD_SEG7	7
#define SLCD_SEG8	8
#define SLCD_SEG9	9
#define SLCD_SEG10	10
#define SLCD_SEG11	11
#define SLCD_SEG12	12
#define SLCD_SEG13	13
#define SLCD_SEG14	14
#define SLCD_SEG15	15
#define SLCD_SEG16	16
#define SLCD_SEG17	17
#define SLCD_SEG18	18
#define SLCD_SEG19	19
#define SLCD_SEG20	20
#define SLCD_SEG21	21
#define SLCD_SEG22	22
#define SLCD_SEG23	23
#define SLCD_SEG24	24
#define SLCD_SEG25	25
#define SLCD_SEG26	26
#define SLCD_SEG27	27
#define SLCD_SEG28	28
#define SLCD_SEG29	29
#define SLCD_SEG30	30
#define SLCD_SEG31	31

#endif //__SWM1800_SLCD_H__
