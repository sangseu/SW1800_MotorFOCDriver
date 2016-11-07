/***********************************************************************
 *                                                                     *
 *    Filename:       park.h                                        *
 *    Date:           10/01/08                                         *
 *                                                                     *
 **********************************************************************/
#ifndef __PARK_H__
#define __PARK_H__
#include "general.h"
//------------------  C API for Park Clarke routines ---------------------

typedef struct 
{
    s16   qAngle;
    s16   qSin;
    s16   qCos;
    s16   qIa;
    s16   qIb;
    s16   qIalpha;
    s16   qIbeta;
    s16   qId;
    s16   qIq;
    s16   qVd;
    s16   qVq;
    s16   qValpha;
    s16   qVbeta;
    s16   qV1;
    s16   qV2;
    s16   qV3;

} tParkParm;

u32 sqrt_16(u32 radicand);
void SinCos(tParkParm *pParkParm);      // Calculate qSin,qCos from iAngle
void ClarkePark(tParkParm *pParkparm);  // Calculate qId,qIq from qCos,qSin,qIa,qIb
void InvPark(tParkParm *pParkparm);     // Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq

extern tParkParm ParkParm;
#endif




