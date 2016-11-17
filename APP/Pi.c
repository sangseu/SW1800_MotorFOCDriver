#include "Pi.h"

void InitPI( tPIParm *pParm)
{
	pParm->qdSum = 0;
	pParm->qOut = 0;
}


void CalcPI( tPIParm *pParm)
{
	//	unsigned int Err,U,Exc;
	s32 Err,Exc;
	s32 U;
#if 0
	Err = pParm->qInRef - pParm->qInMeas;			//Q15
	U = pParm->qdSum + pParm->qKp * Err;	//Q15
	U = U/DIV_RATIO;

	if( U > pParm->qOutMax)
	pParm->qOut = pParm->qOutMax;	//Q15
	else if( U < pParm->qOutMin)
	pParm->qOut = pParm->qOutMin;	//Q15
	else  
	pParm->qOut = U; //Q15
	Exc = U - pParm->qOut;

	pParm->qdSum = pParm->qdSum + (pParm->qKi * Err - pParm->qKc * Exc);//Q15
#endif
	s32 valtemp1;
	Err  = pParm->err = pParm->qInRef - pParm->qInMeas;//pParm->qInMeas - pParm->qInRef;	//			//Q15  

	valtemp1 = pParm->qKp * Err;
	U = pParm->qdSum + RIGHSHIFT15(valtemp1);    //U  = pParm->qdSum + (pParm->qKp * Err)/32767;	//Q15

	if( U > pParm->qOutMax)
	pParm->qOut = pParm->qOutMax;	//Q15 
	else if( U < pParm->qOutMin)
	pParm->qOut = pParm->qOutMin;	//Q15
	else  
	pParm->qOut = U; //Q15

	Exc = pParm->exc = U - pParm->qOut;

	valtemp1 = pParm->qKi * Err - pParm->qKc * Exc;
	pParm->qdSum = pParm->qdSum + RIGHSHIFT15(valtemp1);//pParm->qdSum = pParm->qdSum + ((pParm->qKi * Err - pParm->qKc * Exc))/32767;//Q15BKi
}


//------------------------- PID-----------------------------------------------------------------------------------
int Cal_PID (PID *Parm) 
{
	int du;

	Parm->Kpsum = Parm->Kp*(Parm->err - Parm->er0);
	Parm->Kisum = Parm->Ki*Parm->err;

	du = RIGHSHIFT15(Parm->Kpsum) + RIGHSHIFT15(Parm->Kisum);       //du = ((Parm->Kp*(Parm->err - Parm->er0))>>15)+((Parm->Ki*Parm->err)>>15);  

	Parm->er0 = Parm->err;

	return du;
}

int gabs(int ek)
{
	if(ek<0)
	{
		ek=0-ek;
	}
	return ek;
}

