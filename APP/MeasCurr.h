 /**********************************************************************/
#ifndef __MEASCURR_H__
#define __MEASCURR_H__
//------------------  C API for MeasCurr routines ---------------------

#include "Park.h"
typedef struct 
{
	s16  qKa;        
	s16   Offseta;

	s16  qKb;        
	s16   Offsetb;
} tMeasCurrParm;

#define AD_Vdc   0
#define AD_IPM_Temperature 1
#define AD_Speed 2
#define AD_Ia    3
#define AD_Ib    4

void MeasCompCurr(tParkParm *pParkParm,tMeasCurrParm *pMeaCuParm,s16 AD0,s16 AD1);
void InitMeasCompCurr(tMeasCurrParm *pMeasCurrParm, s16 Offset_a, s16 Offset_b );

extern tMeasCurrParm MeasCurrParm;

#endif

