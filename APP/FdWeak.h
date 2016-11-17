/******************************************************************************/
#ifndef __FDWEAK_H__
#define __FDWEAK_H__

#include "UserParms.h"
#include "general.h"

//------------------  C API for FdWeak routine ---------------------

typedef struct {
	s16	qK1;            // < Nominal speed value
	s16	qIdRef;
	s16	qFwOnSpeed;
	s16	qFwActiv;
	s16	qIndex;
	s16	qFWPercentage;
	s16	qInterpolPortion;
	s32	qFwCurve[16];	// Curve for magnetizing current
} tFdWeakParm;
extern tFdWeakParm FdWeakParm;
s32 FieldWeakening( s32 qMotorSpeed );
void FWInit (void);

#endif




