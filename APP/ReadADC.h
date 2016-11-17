
/**********************************************************************/
#ifndef __READADC_H__
#define __READADC_H__
//------------------  C API for ReadADC routines ---------------------

typedef struct 
{
	s16   qK;         // 1.15 
	s16   qADValue;   // 1.15

} tReadADCParm;

void ReadSignedADC0( tReadADCParm* pParm ); // Returns signed value -2*iK -> 2*iK

#endif

