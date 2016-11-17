
 /**********************************************************************/
#ifndef __PI_H__
#define __PI_H__

#include "general.h"
//------------------  C API for PI routines ---------------------

typedef struct 
{
	s32   qdSum;          //
	s16   qKp;
	s16   qKi;
	s16   qKc;
	s16   qOutMax;
	s16   qOutMin;
	s16   qInRef; 
	s16   qInMeas;
	s16   qOut;

	s32 err;
	s32 exc;
} tPIParm;

typedef struct 
{
	s16 err;
	s16 er0;
	s16 er1;    

	s16 Kp;
	s16 Ki;
	s16 Kd;

	s16 Kp_val;
	s16 Ki_val;

	s32 Kpsum;
	s32 Kisum;

	s16 du;
}PID;

int Cal_PID (PID *Parm);

void InitPI( tPIParm *pParm);
void CalcPI( tPIParm *pParm);
int gabs(int ek);

#endif


