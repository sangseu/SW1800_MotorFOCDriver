#include "MeasCurr.h"

extern s16 ADIa ;			//AD sample Ia value
extern s16 ADIb ;
void InitMeasCompCurr( tMeasCurrParm *pMeasCurrParm, s16 Offset_a, s16 Offset_b )
{
	pMeasCurrParm->Offseta = Offset_a ;
	pMeasCurrParm->Offsetb = Offset_b ;
//     	pMeasCurrParm->Offseta = ADIa = 2048;//2700;//
// 	pMeasCurrParm->Offsetb = ADIb = 2048;//2700;//
}

s16 CorrADC0,CorrADC1;
void MeasCompCurr(tParkParm *pParkParm,tMeasCurrParm *pMeaCuParm,s16 AD0,s16 AD1)
{
    s32 valtemp;
     CorrADC0 = AD0 - pMeaCuParm->Offseta;
  	CorrADC1 = AD1 - pMeaCuParm->Offsetb;
//    CorrADC0 = pMeaCuParm->Offseta - AD0;
// 	CorrADC1 = pMeaCuParm->Offsetb - AD1;
    
    if( CorrADC0 > 2048 )
        CorrADC0 = 2048;
    else if( CorrADC0 < -2048)
        CorrADC0 = -2048;
    if( CorrADC1 > 2048 )
        CorrADC1 = 2048;
    else if( CorrADC1 < -2048)
        CorrADC1 = -2048;
	
	pParkParm->qIa = CorrADC0<<3;//*pMeaCuParm->Offsetb/pMeaCuParm->Offseta;   //AD/2048*32767转化为Q15格式
	pParkParm->qIb = CorrADC1<<3; 

}
