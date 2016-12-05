/****************************************************************************
类型定义
****************************************************************************/
#ifndef __GENERAL_H__
#define __GENERAL_H__

//#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef signed char s8;
typedef signed short s16;
typedef signed int s32;
typedef unsigned long ulong;

typedef unsigned char  bool;

#define INPFRENUM 700

#define LED_ON		GPIOB->DATA &= ~(0x01 << PIN1)        //B1 
#define LED_OFF     GPIOB->DATA |= (0x01 << PIN1)


#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif


#define SetBit(regiset, offset)    		regiset |= 1U << offset
#define ResetBit(regiset, offset)  		regiset &= ~(1U << offset)
#define GetBit(regiset, offset)	   		((regiset >> offset) &0x01)

#define Q15(Float_Value)	\
        ((Float_Value < 0.0) ? (s32)(32768 * (Float_Value) - 0.5) \
        : (s32)(32767 * (Float_Value) + 0.5))
#define Q32(Float_Value)	\
        ((Float_Value < 0.0) ? (s32)(2147483649 * (Float_Value) - 0.5) \
        : (s32)(2147483648 * (Float_Value) + 0.5))
//#define Q15(Float_Value)	(32767 * Float_Value)
#define REFINAMPS(Amperes_Value)	\
		(Amperes_Value * DQKA*RSHUNT*DIFFAMPGAIN/(VDD/2))

#define     RIGHSHIFT15(Int_Value)   ( (Int_Value>=0) ? (Int_Value>>15) : (-((-Int_Value)>>15)) )    //对负数右移位，取绝对值，然后移位，再取相反数
#define     RIGHSHIFT10(Int_Value)   ( (Int_Value>=0) ? (Int_Value>>10) : (-((-Int_Value)>>10)) )
#define     RIGHSHIFT1(Int_Value)   ( (Int_Value>=0) ? (Int_Value>>1) : (-((-Int_Value)>>1)) )
#define     RIGHSHIFT2(Int_Value)   ( (Int_Value>=0) ? (Int_Value>>2) : (-((-Int_Value)>>2)) )
#define     RIGHSHIFT7(Int_Value)   ( (Int_Value>=0) ? (Int_Value>>7) : (-((-Int_Value)>>7)) )


// Main functions prototypes
void SetupPorts( void );
bool SetupParm(void);
void DoControl( void );
void CalculateParkAngle(void);
void SetupControlParameters(void);
void DebounceDelay(void);
s32 VoltRippleComp(s32 Vdq1);
void DIV_Fun(int Divd, int Divs, int *Quo, int *Rem);   //除法函数




#endif

