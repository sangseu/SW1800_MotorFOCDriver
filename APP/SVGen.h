 /***********************************************************************                                                                    *
 *    Filename:       

 .h                                          *
 *    Date:           10/01/08                                         *
 **********************************************************************/
#ifndef __SVGEN_H__
#define __SVGEN_H__
//------------------  C ASVGen for SVGen routines ---------------------

#include "Park.h"
#include "General.h"

typedef struct 
{
    s16   iPWMPeriod;

    s16   qVr1;		
    s16   qVr2;		
    s16   qVr3;		
} tSVGenParm;

void CalcRefVec(tParkParm *pParkparm,tSVGenParm *pSVGenParm);
void CalcSVGen( tSVGenParm *pSVGenParm );
void CorrectPhase( void );

void CalcTimes(tSVGenParm *pSVGenParm,s16 t1,s16 t2);

extern tSVGenParm SVGenParm;

void Svpwm_Duty(s16 angle,s16 uout,int *t0,int *t1,int *t2);
void Svpwm_Cal(s16 qua,int *t0,int *t1,int *t2);
int dutypid (int err,int *er0,int *er1);

#endif
