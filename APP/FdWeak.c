/******************************************************************************/
#include "FdWeak.h"
#include "general.h"
#include "smc.h"

tFdWeakParm	FdWeakParm;

s32 FieldWeakening(s32 qMotorSpeed)
{
	s32 Di, Vi, Qi, Ri;
	/* if the speed is less than one for activating the FW */
	if (qMotorSpeed <= FdWeakParm.qFwOnSpeed)
	{
	/* set Idref as first value in magnetizing curve */
		FdWeakParm.qIdRef = FdWeakParm.qFwCurve[0];
	} 
	else
	{
		// Index in FW-Table. The result is left shifted 11 times because
		// we have a field weakening table of 16 (4 bits) values, and the result
		// of the division is 15 bits (16 bits, with no sign). So
		// Result (15 bits) >> 11 -> Index (4 bits).
		//		FdWeakParm.qFWPercentage = FracDiv((qMotorSpeed-FdWeakParm.qFwOnSpeed), \
		//							   Q15(OMEGAFIELDWK-OMEGANOMINAL+1));
		Di = qMotorSpeed-FdWeakParm.qFwOnSpeed;
		Vi = OMEGAFIELDWK-OMEGANOMINAL;
		DIV_Fun(Di, Vi, &Qi, &Ri);
		FdWeakParm.qFWPercentage = Qi;
		//		FdWeakParm.qFWPercentage = \
		//		(qMotorSpeed-FdWeakParm.qFwOnSpeed)/(OMEGAFIELDWK-OMEGANOMINAL);

		FdWeakParm.qIndex = FdWeakParm.qFWPercentage >> 11;

		// Interpolation betwen two results from the Table. First mask 11 bits,
		// then left shift 4 times to get 15 bits again.
		FdWeakParm.qInterpolPortion = (FdWeakParm.qFWPercentage & 0x07FF) ;

		//		FdWeakParm.qIdRef = FdWeakParm.qFwCurve[FdWeakParm.qIndex] \
		//							- FracMpy(FdWeakParm.qFwCurve[FdWeakParm.qIndex] \
		//									- FdWeakParm.qFwCurve[FdWeakParm.qIndex+1] \
		//									 ,FdWeakParm.qInterpolPortion);
		FdWeakParm.qIdRef = FdWeakParm.qFwCurve[FdWeakParm.qIndex] \
		- (((FdWeakParm.qFwCurve[FdWeakParm.qIndex] \
		- FdWeakParm.qFwCurve[FdWeakParm.qIndex+1])\
		*FdWeakParm.qInterpolPortion)>>12);

	}
	return FdWeakParm.qIdRef;
}

void FWInit(void)
{
	/* initialize magnetizing curve values */
	FdWeakParm.qFwOnSpeed = Q15(OMEGANOMINAL);
	FdWeakParm.qFwCurve[0]	= dqKFw0;
	FdWeakParm.qFwCurve[1]	= dqKFw1;
	FdWeakParm.qFwCurve[2]	= dqKFw2;
	FdWeakParm.qFwCurve[3]	= dqKFw3;
	FdWeakParm.qFwCurve[4]	= dqKFw4;
	FdWeakParm.qFwCurve[5]	= dqKFw5;
	FdWeakParm.qFwCurve[6]	= dqKFw6;
	FdWeakParm.qFwCurve[7]	= dqKFw7;
	FdWeakParm.qFwCurve[8]	= dqKFw8;
	FdWeakParm.qFwCurve[9]	= dqKFw9;
	FdWeakParm.qFwCurve[10]	= dqKFw10;
	FdWeakParm.qFwCurve[11]	= dqKFw11;
	FdWeakParm.qFwCurve[12]	= dqKFw12;
	FdWeakParm.qFwCurve[13]	= dqKFw13;
	FdWeakParm.qFwCurve[14]	= dqKFw14;
	FdWeakParm.qFwCurve[15]	= dqKFw15;	
	return;
}

