#ifndef __SWM1800_CMP_H__
#define	__SWM1800_CMP_H__


#define CMP0	0
#define CMP1	1
#define CMP2	2


void CMP_Init(uint32_t CMPx, uint32_t inp_ex, uint32_t isHYS, uint32_t int_en);	//CMP模拟比较器初始化
void CMP_Open(uint32_t CMPx);						//开启比较器
void CMP_Close(uint32_t CMPx);						//关闭比较器

uint32_t CMP_Output(uint32_t CMPx);

void CMP_SetVRef(uint32_t volt);					//设置内部参考电压

#define CMP_VREF_0V30	0
#define CMP_VREF_0V45	1
#define CMP_VREF_0V60	2
#define CMP_VREF_0V75	3
#define CMP_VREF_0V90	4
#define CMP_VREF_1V05	5
#define CMP_VREF_1V20	6
#define CMP_VREF_1V35	7
#define CMP_VREF_1V50	8
#define CMP_VREF_1V65	9
#define CMP_VREF_1V80	10
#define CMP_VREF_1V95	11
#define CMP_VREF_2V10	12
#define CMP_VREF_2V25	13
#define CMP_VREF_2V40	14
#define CMP_VREF_2V55	15


void CMP_INTEn(uint32_t CMPx);
void CMP_INTDis(uint32_t CMPx);
void CMP_INTClr(uint32_t CMPx);
uint32_t CMP_INTStat(uint32_t CMPx);


#endif //__SWM1800_CMP_H__
