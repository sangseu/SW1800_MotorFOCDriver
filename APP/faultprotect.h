#ifndef _FAULTPROTECT_H
#define _FAULTPROTECT_H

#include "general.h"
#include "UserParms.h"
#include "Park.h"
#include "SWM1800.h"
#include "MeasCurr.h"
#include "Pi.h"

#define DELTA_VALUE 20		//
#define IPM_FO_LOW_TIMES    20  //连续检测到FO引脚输出低电平次数
#define OVER_TEMPERATURE_TIMES  20  //连续检测到过温次数
#define UNNORMAL_SPEED_TIMES    1   //连续检测到转速不对次数

#define R_VBUS_RATIO		121     //母线电压电阻分压比例
#define DCbus_OVERVOLTAGE 320	//单位为V
#define DCbus_UNDERVOLTAGE 280	//母线电压大概250V为欠压
#define DCbus_OVERVOLTAGE_ADVAL (DCbus_OVERVOLTAGE*4095/(R_VBUS_RATIO*VDD))  //4000	// 3.2v左右过压保护点，母线电压大概400V ,需要修改电路的电阻分压
#define DCbus_UNDERVOLTAGE_ADVAL (DCbus_UNDERVOLTAGE*4095/(R_VBUS_RATIO*VDD))   //欠压AD值

#define PHASE_CURRENT_DEVIATION_RATIO   10  //相电流偏差值比例
#define PHASE_CURRENT_AD_DEVIATION_RATIO   10  //相电流AD偏差值比例

#define PHASE_OVERCURRENT_VALUE 0.8	//过流电流，单位为A
#define CALCULATE_PHASE_CURRENT(x) ((x*RSHUNT*DIFFAMPGAIN*4095)/VDD)
#define PHASE_OVERCURRENT (CALCULATE_PHASE_CURRENT(PHASE_OVERCURRENT_VALUE))      // 过流点对应的AD值

#define PHASE_MINCURRENT_VALUE 0.2		//无电机时相电流，单位为A
#define PHASE_MINCURRENT_VALUE_TURN_AD (CALCULATE_PHASE_CURRENT(PHASE_MINCURRENT_VALUE)) 
#define PHASECURRENT_NEAR_ZERO_ADVALUE (0.25*PHASE_MINCURRENT_VALUE_TURN_AD)

#define MOTOR_MINROTATE 300
#define ELECTRIC_TIMER_ADSAMPLE_NUMBER (PWMFREQUENCY*60/(MOTOR_MINROTATE*POLEPAIRS)) 

/***********************过温检测参数定义************************/
#define RESISTOR_DEG25		47000	    //IPM温度25度时，内阻为47k
#define RESISTOR_DEG100	    2900		//IPM温度100度时，内阻为2.9k
#define RESISTOR_PULLUP		8200		//IPM测温上拉电阻
#define OVER_TEMPERATURE    95		    //设置IPM过温保护，单位为度
#define ADVALUE_DEG25		RESISTOR_DEG25*4095/(RESISTOR_DEG25+RESISTOR_PULLUP)	//25度时对应的AD值
#define ADVALUE_DEG100		RESISTOR_DEG100*4095/(RESISTOR_DEG100+RESISTOR_PULLUP)  //100度时对应的AD值
#define RESISTOR_DEGOVERTEMP ((OVER_TEMPERATURE-25)*(RESISTOR_DEG100-RESISTOR_DEG25)/(100-25)+RESISTOR_DEG25)	//过温时IPM内阻
#define ADVALUE_OVER_TEMPERATUR	    RESISTOR_DEGOVERTEMP*4095/(RESISTOR_DEGOVERTEMP+RESISTOR_PULLUP)	//过温时对应的AD值
/****************************************************************************/


/***********************************系统状态*************************************************/
#define SYSTATUS_STANDBY                1       //待机
#define SYSTATUS_NORMAL                 2       //正常
#define SYSTATUS_OVERVOL                3       //过压
#define SYSTATUS_LOWVOL                 4       //欠压
#define SYSTATUS_OVERPHASECURRENT       5       //相电流过流
#define SYSTATUS_LOSESTEP               6       //失步
#define SYSTATUS_SPEEDUNNORMAL          7       //转速不对
#define SYSTATUS_CURRENTDEVIATION       8       //电流偏差过大
#define SYSTATUS_IPM_FO                 9       //FO输入故障
#define SYSTATUS_ADDEVIATION            10      //AD异常
#define SYSTATUS_OVERTEMPERATURE        11      //过温
/***************************************************************************************************/

/***********************************故障LED亮灭时间*************************************************/
#define STANDBY_ONTIME      10      //待机LED亮时间，10*100ms
#define STANDBY_OFFTIME     (STANDBY_ONTIME+10)      //待机LED灭时间，10*100ms
#define NORMAL_ONTIME       6       //待机LED亮时间，3*100ms
#define NORMAL_OFFTIME      (NORMAL_ONTIME+6)       //待机LED灭时间，3*100ms
#define FAULT_TWINKLETIME        2       //故障LED闪烁时间，3*100ms
#define FAULT_OFFTIME       (15-FAULT_TWINKLETIME)      //故障LED灭时间，15*100ms
/***************************************************************************************************/

/***********************************故障LED闪烁次数*************************************************/
#define OVER_VOL_TWINKLETIMES                   1   //直流母线输入过压
#define LOW_VOL_TWINKLETIMES                    2   //直流母线输入电压欠压
#define OVER_PHASECURRENT_TWINKLETIMES          3   //相电流输出过电流
#define LOSE_STEP_TWINKLETIMES                  4   //失步检出(无电机或缺相)
#define SPEED_UNNORMAL_TWINKLETIMES             5   //3相输出缺相(转速不对)
#define CURRENT_DEVIATION_TWINKLETIMES          6   //3相输出缺相(检测电流偏差过大)
#define IPM_FO_TWINKLETIMES                     7   //逆变器IPM_FO故障
#define AD_DEVIATION_TWINKLETIMES               8   //AD异常检测故障(电流检测点电平偏差过大)
#define OVER_TEMPERATURE_TWINKLETIMES           9   //温度保护(模块过温保护)
/***************************************************************************************************/

#define GET_IPM_FO_STATUS ((GPIOE->DATA >> PIN3) & 0x01)    //获取IPM_FO引脚电平

typedef struct
{
	u32 Ad_Temperature_total;
	u32 Ad_Temperature_times;
	u32 Ad_Temperature_value;
}IPM;

typedef struct
{
	u8 Systemstatus;	//系统状态，1=待机，2=正常运转，>=3故障
	u32 Timernumcount;	//定时器时间累加器
	u32 Twinkletimes;	//闪烁次数
}LED_INDICATE;

typedef struct
{
	s16 Ia_max; 	//A相电流AD最大值
	s16 Ia_min;
	s16 Ib_max; 	//B相电流AD最大值
	s16 Ib_min;
	u32 Adsample_times; 	//AD取样数值
	u16 Phase_current_check_flag;	//相电流幅值检测完成标志, 0=未完成，1＝完成
}PHASE_CURRENT;



typedef struct 
{

	u32 fault_code;

	IPM *IPM_module;

	LED_INDICATE *Led_indicate;

	PHASE_CURRENT *Phase_current_detect;
	
}MOTOR_FAULT;

extern s16 TargetDCbus;
extern s16 CorrADC0;
extern s16 CorrADC1;
extern u32 fg_outfrequence;
extern s16 View_Variable1,View_Variable2,View_Variable3,View_Variable4;


void phase_current_max_check(MOTOR_FAULT*pmotor_fault);
    
void motor_fault_detect(MOTOR_FAULT*pmotor_fault);

void SysLed_Twinkle(MOTOR_FAULT*pmotor_fault);

void motor_fault_init(MOTOR_FAULT *pmotor_fault);


#endif
