#include "Park.h"

//#define OneBySq3 0.577 
 #define OneBySq3 591   //18906		//0.577*32767=18906
 //#define OneBySq3 0x49E7   // 1/sqrt(3) in 1.15 format
/**********************************************************************
ClarkePark

Description:        
 Calculate Clarke & Park transforms. 
 Assumes the Cos and Sin values are in qSin & qCos.

	Ialpha = Ia
	Ibeta  = Ia*dOneBySq3 + 2*Ib*dOneBySq3;
	where Ia+Ib+Ic = 0

	Id =  Ialpha*cos(Angle) + Ibeta*sin(Angle)
	Iq = -Ialpha*sin(Angle) + Ibeta*cos(Angle)
**********************************************************************/
// extern unsigned int MeasComp,Calculate,Clarke,DoCon,Sin,Inv,CalcRef,CalcSVG;
void ClarkePark(tParkParm *pParkparm)
{
	s32 valtemp;
	s16 SinW,CosW,IalphaW,IbetaW;
	//    s32 Di, Vi, Qi, Ri;

	SinW = pParkparm->qSin;
	CosW = pParkparm->qCos;
	IalphaW = pParkparm->qIa;

	valtemp = (pParkparm->qIa  + pParkparm->qIb *2) * OneBySq3 ;
	IbetaW = RIGHSHIFT10(valtemp); //IbetaW = ((pParkparm->qIa  + ((pParkparm->qIb) *2)) * OneBySq3 )/1024;

	pParkparm->qIalpha = IalphaW;    //
	pParkparm->qIbeta = IbetaW;	 //

	valtemp = (SinW * IbetaW) + (CosW * IalphaW);
	pParkparm->qId = RIGHSHIFT15(valtemp);//pParkparm->qId = ((SinW * IbetaW) + (CosW * IalphaW))/DIV_RATIO;	 //Q15*Q15=Q30,所以要右移15位转变为Q15

	valtemp = (CosW * IbetaW) - (SinW * IalphaW);
	pParkparm->qIq = RIGHSHIFT15(valtemp);//pParkparm->qIq = ((CosW * IbetaW) - (SinW * IalphaW))/DIV_RATIO;	
}


