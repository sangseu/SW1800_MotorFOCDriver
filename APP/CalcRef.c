#include "Park.h"
#include "Svgen.h"

//#define Sq3OV2 0.866  //
#define Sq3OV2 887		//0.866*1024
//#define Sq3OV2	0x6ED9  //sqrt(3)/2 in 1.15 format
/***********************************************************************
                                                             
	Filename:       CalcRef.s                                
	Date:           10/01/08                                     
	                                               
	CalcRefVec

	Description:        
	Calculate the scaled reference vector, (Vr1,Vr2,Vr3), from qValpha,qVbeta.
	The method is an modified inverse Clarke transform where Valpha & Vbeta 
	are swapped compared to the normal Inverse Clarke.

	Vr1 = Vbeta
	Vr2 = (-Vbeta/2 + sqrt(3)/2 * Valpha)
	Vr3 = (-Vbeta/2 - sqrt(3/2) * Valpha)

	Functional prototype:
	void CalcRefVec(tParkParm *pParkparm)
**********************************************************************/	
// extern u32 MeasComp,Calculate,Clarke,DoCon,Sin,Inv,CalcRef,CalcSVG;

void CalcRefVec(tParkParm *pParkparm,tSVGenParm *pSVGenParm)
{
	s32 valtemp1; 
    s16 ValphaW = pParkparm->qValpha;
	s16 VbetaW = pParkparm->qVbeta;
	
	
	pSVGenParm->qVr1 = VbetaW ;//Q15
    valtemp1 = Sq3OV2 * ValphaW;
    pSVGenParm->qVr2 = RIGHSHIFT10(valtemp1) - RIGHSHIFT1(VbetaW); //pSVGenParm->qVr2 = ((Sq3OV2 * ValphaW)/1024) - (VbetaW /2);     //pSVGenParm->qVr2 = ((Sq3OV2 * ValphaW)>>10) - (VbetaW>>1);// 		//Q15	
    valtemp1 = -Sq3OV2 * ValphaW;
    pSVGenParm->qVr3 = RIGHSHIFT10(valtemp1) - RIGHSHIFT1(VbetaW); //pSVGenParm->qVr3 = ((0-Sq3OV2 * ValphaW)/1024)  - (VbetaW /2);  //pSVGenParm->qVr3 = ((0-Sq3OV2 * ValphaW)>>10)  - (VbetaW>>1);// 		//Q15
}

