#ifndef __PARMS_H__
#define __PARMS_H__

typedef struct 
{
	u32 LockTime;
	long EndSpeed;

}tMotorParm;

extern tMotorParm MotorParm;

void InitMotorParm(void);

#endif

