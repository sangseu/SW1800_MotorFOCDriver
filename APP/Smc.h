/***********************************************************************
 *                                                                     *
 *    Filename:       smc.h                                         *
 *    Date:           13/04/2015                                         *
 *                                                                     *                   
 *                                                                     *
 **********************************************************************/
#ifndef __SMC_H__
#define __SMC_H__

#include "UserParms.h"


typedef struct	
{  
	s16  Valpha;   		// Input: Stationary alfa-axis stator voltage 
	s16  Ealpha;   		// Variable: Stationary alfa-axis back EMF 
	s16  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
	s16  Zalpha;      	// Output: Stationary alfa-axis sliding control 
	s16  Gsmopos;    	// Parameter: Motor dependent control gain 
	s16  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current 
	s16  Fsmopos;    	// Parameter: Motor dependent plant matrix 
	s16  Vbeta;   		// Input: Stationary beta-axis stator voltage 
	s16  Ebeta;  		// Variable: Stationary beta-axis back EMF 
	s16  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
	s16  Zbeta;      	// Output: Stationary beta-axis sliding control 
	s16  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current 
	s16  Ialpha;  		// Input: Stationary alfa-axis stator current 
	s16  IalphaError; 	// Variable: Stationary alfa-axis current error                 
	s16  Kslide;     	// Parameter: Sliding control gain 
	s16  MaxSMCError;  	// Parameter: Maximum current error for linear SMC 
	s16  Ibeta;  		// Input: Stationary beta-axis stator current 
	s16  IbetaError;  	// Variable: Stationary beta-axis current error                 
	s16  Kslf;       	// Parameter: Sliding control filter gain 
	s16  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
	s16  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
	s16  ThetaOffset;	// Output: Offset used to compensate rotor angle
	s16  Theta;			// Output: Compensated rotor angle 
	s16  Omega;     	// Output: Rotor speed
	s16  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
} SMC;	            

typedef SMC *SMC_handle;

#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

// Define this in RPMs
#define SPEED_CUF 500
    
#define SPEED0 MINSPEEDINRPM
#define SPEED1 (SPEED0 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED2 (SPEED1 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED3 (SPEED2 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED4 (SPEED3 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED5 (SPEED4 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED6 (SPEED5 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED7 (SPEED6 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED8 (SPEED7 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED9 (SPEED8 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED10 (FIELDWEAKSPEEDRPM)

// Define this in Degrees, from 0 to 360

#define THETA_AT_ALL_SPEED 110
    
#define OMEGA_CUF (float)(SPEED_CUF * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
    
#define OMEGA0 (float)(SPEED0 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA1 (float)(SPEED1 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA2 (float)(SPEED2 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA3 (float)(SPEED3 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0  / 60.0)
#define OMEGA4 (float)(SPEED4 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0  / 60.0)
#define OMEGA5 (float)(SPEED5 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0  / 60.0)
#define OMEGA6 (float)(SPEED6 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0  / 60.0)
#define OMEGA7 (float)(SPEED7 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0  / 60.0)
#define OMEGA8 (float)(SPEED8 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0  / 60.0)
#define OMEGA9 (float)(SPEED9 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0  / 60.0)
#define OMEGA10 (float)(SPEED10 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0  / 60.0)
								
#define OM0_OFFSET	6000
#define OM1_OFFSET	6500
#define OM2_OFFSET	7000
#define OM3_OFFSET	7500
#define OM4_OFFSET	8000
#define OM5_OFFSET	8500

#define OFFSET0	-2000
#define OFFSET1	-3000
#define OFFSET2	-4000
#define OFFSET3	-5000
#define OFFSET4	-6000
#define OFFSET5	-6000




#define OMEGANOMINAL	(float)(NOMINALSPEEDINRPM * LOOPTIMEINSEC * \
                		IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGAFIELDWK	(float)(FIELDWEAKSPEEDRPM * LOOPTIMEINSEC * \
                		IRP_PERCALC * POLEPAIRS * 2.0  / 60.0)

//  #define THETA_ALL (float)(THETA_AT_ALL_SPEED  /180.0 )
#define THETA_ALL (float)(THETA_AT_ALL_SPEED * 32768.0 /180.0 )

//#define THETA_ALL (float)(THETA_AT_ALL_SPEED )

//#define CONSTANT_PHASE_SHIFT Q15(THETA_ALL)
#define CONSTANT_PHASE_SHIFT (THETA_ALL)

#define _PI 3.1416     //3.1416=25736/8192
#define PI_INT  25736

void SMC_Position_Estimation(SMC_handle);
void SMCInit(SMC_handle);
void CalcEstI(SMC_handle);
void CalcIError(SMC_handle);
void CalcZalpha(SMC_handle);
void CalcZbeta(SMC_handle);
void CalcBEMF(SMC_handle);
void CalcOmegaFltred(SMC_handle);
s16 FracMpy(s16 mul_1, s16 mul_2);
s16 FracDiv(s16 num_1, s16 den_1);

extern s16 PrevTheta;
extern s16 AccumTheta;
extern u32 AccumThetaCnt;

#endif

