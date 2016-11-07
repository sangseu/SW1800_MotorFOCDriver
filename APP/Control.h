#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "general.h"

//------------------  C API for Control routine ---------------------

typedef struct 
{
	s16 qVelRef;    // Reference velocity
	s16 qVdRef;     // Vd flux reference value
	s16 qVqRef;     // Vq torque reference value
} tCtrlParm;

tCtrlParm CtrlParm;

#endif
