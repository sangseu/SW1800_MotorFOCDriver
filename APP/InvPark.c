#include "Park.h"
#include "General.h"

/*******************************************
Description:        
	Calculate the inverse Park transform. Assumes the Cos and Sin values 
	are in the ParkParm structure.

	Valpha =  Vd*cos(Angle) - Vq*sin(Angle)
	Vbeta  =  Vd*sin(Angle) + Vq*cos(Angle)
********************************************/
// s32 VdW,VqW;

void InvPark(tParkParm *pParkparm)
{
	s32 valtemp;
	s16 VdW = pParkparm->qVd;
	s16 VqW = pParkparm->qVq;
	s16 SinW = pParkparm->qSin;
	s16 CosW = pParkparm->qCos;

	valtemp = VdW * CosW- SinW * VqW;
	pParkparm->qValpha= RIGHSHIFT15(valtemp);//     pParkparm->qValpha= (VdW * CosW- SinW * VqW)>>15;// 	pParkparm->qValpha= (VdW * CosW- SinW * VqW)/DIV_RATIO;

	valtemp = VdW * SinW + CosW * VqW;
	pParkparm->qVbeta = RIGHSHIFT15(valtemp);//     pParkparm->qVbeta = (VdW * SinW + CosW * VqW)>>15;// 	pParkparm->qVbeta = (VdW * SinW + CosW * VqW)/DIV_RATIO;

}
