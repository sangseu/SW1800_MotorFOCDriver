/***********************************************************************
 *                                                                     *
 *    Filename:       UserParms.h                                      *
 *    Date:           13/04/2015                                         *
 *                                                                     *
 *                                                                     *
 **********************************************************************/
#ifndef __USERPARMS_H__
#define __USERPARMS_H__

#define _0_05DEG 	1//9	// The value for 0.05 degrees is converted
					// .05 * 32768 / 180 = 9.1, approx 9.
                    



//************** Start-Up Parameters **************
#define ZHIGAO_MOTOR

#define LOCKTIMEINSEC  4//0.25		// Initial rotor lock time in seconds
								// Make sure LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC)
								// is less than 65535.
#define OPENLOOPTIMEINSEC 8//5.0	// Open loop time in seconds. This is the time that
								// will take from stand still to closed loop.
								// Optimized to overcome the brake inertia.
								// (Magtrol AHB-3 brake inertia = 6.89 kg x cm2).
#define INITIALTORQUE 	0.6       	//0.4 //	 1.0		// Initial Torque demand in Amps.
								// Enter initial torque demand in Amps using REFINAMPS() 
								// macro. Maximum Value for reference is defined by 
								// shunt resistor value and differential amplifier gain.
								// Use this equation to calculate maximum torque in 
								// Amperes:
								// 
								// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
								//
								// For example:
								//
								// RSHUNT = 0.005
								// VDD = 3.3
								// DIFFAMPGAIN = 75
								//
								// Maximum torque reference in Amps is:
								//
								// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
#define ENDSPEEDOPENLOOP MINSPEEDINRPM

// Values used to test Hurst Motor "NT Dynamo DMB0224C10002" at 24VDC input. Motor datasheet at www.hurstmfg.com
//#define POLEPAIRS      	4       // Number of pole pairs
//#define PHASERES		((float)131.0)	// Phase resistance in Ohms.
//#define PHASEIND		((float)0.078)// Phase inductance in Henrys.

//志高电机
#ifdef ZHIGAO_MOTOR
#define POLEPAIRS      	4      // Number of pole pairs
#define PHASERES		((float)11.5)	// Phase resistance in Ohms.
#define PHASEIND		((float)0.056)// Phase inductance in Henrys.
#else
#define POLEPAIRS      	5       // Number of pole pairs
#define PHASERES		((float)2)//((float)1.5)//*(1.5/24.0))	// Phase resistance in Ohms.
#define PHASEIND		((float)0.0015)//*(1.5/24.0))// Phase inductance in Henrys.
#endif


#define NOMINALSPEEDINRPM 550//375	// Make sure NOMINALSPEEDINRPM generates a MAXOMEGA < 1.0
								// Use this formula:
								// MAXOMEGA = NOMINALSPEEDINRPM*SPEEDLOOPTIME*POLEPAIRS*2/60
								// If MAXOMEGA > 1.0, reduce NOMINALSPEEDINRPM or execute
								// speed loop faster by reducing SpeedLoopTime.
								// Maximum position of POT will set a reference of 
								// NOMINALSPEEDINRPM.
#define MINSPEEDINRPM	400 	// Minimum speed in RPM. Closed loop will operate at this
								// speed. Open loop will transition to closed loop at
								// this minimum speed. Minimum POT position (CCW) will set
								// a speed reference of MINSPEEDINRPM
#define FIELDWEAKSPEEDRPM  900//480 	// Make sure FIELDWEAKSPEEDRPM generates a MAXOMEGA < 1.0
								// Use this formula:
								// MAXOMEGA = FIELDWEAKSPEEDRPM*SPEEDLOOPTIME*POLEPAIRS*2/60
								// If MAXOMEGA > 1.0, reduce FIELDWEAKSPEEDRPM or execute
								// speed loop faster by reducing SpeedLoopTime.
								// Maximum position of POT will set a reference of 
								// FIELDWEAKSPEEDRPM.
								
/*
// Values used to test Shinano Kenshi Motor "LA052-040E" at 24VDC input. Motor datasheet at www.shinano.com
#define POLEPAIRS      			2
#define PHASERES				((float)0.60)	// Phase resistance in Ohms.
#define PHASEIND				((float)0.0022)// Phase inductance in Henrys.
#define NOMINALSPEEDINRPM 		2700
#define MINSPEEDINRPM			500
#define FIELDWEAKSPEEDRPM 		5300
*/
//************** PWM and Control Timing Parameters **********

#define PWMFREQUENCY	13000		// PWM Frequency in Hertz
#define DEADTIMESEC		0.000001	// Deadtime in seconds, Max value(second) = 63*PWM_DIV/FOSC              
#define	BUTPOLLOOPTIME	0.100		// Button polling loop period in sec
#define SPEEDLOOPFREQ	1000		// Speed loop Frequency in Hertz. This value must
									// be an integer to avoid pre-compiler error

//************** Slide Mode Controller Parameters **********

#define SMCGAIN		     0.5//0.1//0.7//0.915//0.16//0.305//0.5		// Slide Mode Controller Gain (0.0 to 0.9999)
#define MAXLINEARSMC    0.01//0.09//0.1//0.061//0.05		// If measured current - estimated current
								// is less than MAXLINEARSMC, the slide mode
								// Controller will have a linear behavior
								// instead of ON/OFF. Value from (0.0 to 0.9999)
//************** Hardware Parameters ****************

#define RSHUNT			0.2//0.132//0.33	// Value in Ohms of shunt resistors used.
#define DIFFAMPGAIN		4.4//5.1		// Gain of differential amplifier.
#define VDD				3.3		// VDD voltage, only used to convert torque
								// reference from Amps to internal variables
#define SPEEDDELAY 6 // Delay for the speed ramp.
					  // Necessary for the PI control to work properly at high speeds.

//*************** Optional Modes **************
//#define TORQUEMODE
//#define ENVOLTRIPPLE

//************** PI Coefficients **************

//******** D Control Loop Coefficients *******
#define     DKP        Q15(0.1)//Q15(0.2)//Q15(0.0915)//Q15(0.02)//Q15(0.03)//Q15(0.15)//Q15(0.05)	
#define     DKI        Q15(0.02)//Q15(0.00915)//Q15(0.0061)//Q15(0.0061)//Q15(0.005)//Q15(0.01)
#define     DKC        Q15(0.99999)
#define     DOUTMAX    Q15(0.5)

//******** Q Control Loop Coefficients *******
#define     QKP        Q15(0.1)//Q15(0.1)//Q15(0.03)//Q15(0.037)//Q15(0.03)////Q15(0.08)//Q15(0.05)
#define     QKI        Q15(0.02)//Q15(0.025)//Q15(0.00915)//Q15(0.0061)//Q15(0.001)//Q15(0.01)
#define     QKC        Q15(0.99999)
#define     QOUTMAX    Q15(0.9)

//*** Velocity Control Loop Coefficients *****
#define     WKP        Q15(0.05)//Q15(0.0244)//Q15(0.12)
#define     WKI        Q15(0.01)//Q15(0.0061)//
#define     WKC        Q15(0.99999)
#define     WOUTMAX    Q15(0.9)

//************** ADC Scaling **************
// Scaling constants: Determined by calibration or hardware design. 
#define     DQK        Q15((OMEGA10 - OMEGA1)/2.0)	// POT Scaling   扩大65536倍
#ifdef ZHIGAO_MOTOR
#define     DQKA       Q15(0.5)//Q15(0.5)	// Current feedback software gain
#define     DQKB       Q15(0.85)//Q15(0.5)	// Current feedback software gain
#else
#define     DQKA       Q15(0.5)	// Current feedback software gain
#define     DQKB       Q15(0.85)	// Current feedback software gain
#endif

//************** Field Weakening **************
// Enter flux demand Amperes using REFINAMPS() macro. Maximum Value for
// reference is defined by shunt resistor value and differential amplifier gain.
// Use this equation to calculate maximum torque in Amperes:
// 
// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
//
// For example:
//
// RSHUNT = 0.005
// VDD = 3.3
// DIFFAMPGAIN = 75
//
// Maximum torque reference in Amps is:
//
// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
//
// in order to have field weakening, this reference value should be negative,
// so maximum value in this example is -4.4, or REFINAMPS(-4.4)
//#define     dqKFw0  REFINAMPS(0) //3000
//#define     dqKFw1  REFINAMPS(-0.435) //3166
//#define     dqKFw2  REFINAMPS(-0.650) //3333
//#define     dqKFw3  REFINAMPS(-0.915) //3500
//#define     dqKFw4  REFINAMPS(-1.075) //3666
//#define     dqKFw5  REFINAMPS(-1.253) //3833
//#define     dqKFw6  REFINAMPS(-1.432) //4000
//#define     dqKFw7  REFINAMPS(-1.670) //4166
//#define     dqKFw8  REFINAMPS(-1.838) //4333
//#define     dqKFw9  REFINAMPS(-1.952) //4500
//#define     dqKFw10  REFINAMPS(-2.042) //4666
//#define     dqKFw11  REFINAMPS(-2.064) //4833
//#define     dqKFw12  REFINAMPS(-2.100) //5000
//#define     dqKFw13  REFINAMPS(-2.100) //5166
//#define     dqKFw14  REFINAMPS(-2.100) //5333
//#define     dqKFw15  REFINAMPS(-2.100) //5500
#define     dqKFw0  REFINAMPS(0) //3000
#define     dqKFw1  REFINAMPS(-0.1) //3166
#define     dqKFw2  REFINAMPS(-0.1) //3333
#define     dqKFw3  REFINAMPS(-0.1) //3500
#define     dqKFw4  REFINAMPS(-0.1) //3666
#define     dqKFw5  REFINAMPS(-0.1) //3833
#define     dqKFw6  REFINAMPS(-0.1) //4000
#define     dqKFw7  REFINAMPS(-0.1) //4166
#define     dqKFw8  REFINAMPS(-0.1) //4333
#define     dqKFw9  REFINAMPS(-0.1) //4500
#define     dqKFw10  REFINAMPS(-0.15) //4666
#define     dqKFw11  REFINAMPS(-0.15) //4833
#define     dqKFw12  REFINAMPS(-0.15) //5000
#define     dqKFw13  REFINAMPS(-0.15) //5166
#define     dqKFw14  REFINAMPS(-0.15) //5333
#define     dqKFw15  REFINAMPS(-0.15) //5500


//************** Derived Parameters ****************

#define FOSC		(48000000)	// Clock frequency (Hz)
#define DTCY        (1.0/FOSC)		// Instruction cycle period (sec)
#define DDEADTIME   (unsigned int)(DEADTIMESEC*FOSC)	// Dead time in dTcys
#define LOOPTIMEINSEC (1.0/PWMFREQUENCY) // PWM Period = 1.0 / PWMFREQUENCY
#define IRP_PERCALC (unsigned int)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define LOOPINTCY	(LOOPTIMEINSEC/DTCY)   // Basic loop period in units of Tcy
#define LOCKTIME	(unsigned int)(LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC))

#define CURRENTLOOPFREQ 2000  // Current loop Frequency in Hertz. This value must
									// be an integer to avoid pre-compiler error
#define CURRENTLOOPTIME (float)(1.0/CURRENTLOOPFREQ) // Current Control Period
#define IRP_CURRENT_PERCALC (unsigned int)(CURRENTLOOPTIME/LOOPTIMEINSEC)	// PWM loops per current calculation

/* Time it takes to ramp from zero to MINSPEEDINRPM. Time represented in seconds
DELTA_STARTUP_RAMP(角度增量△θ)定义原理:
在开环时间内将电机速度增加到给定的速度值MINSPEEDINRPM,
首先将速度值转换成角度值θ,假设MINSPEEDINRPM转速对应的角度
值为θn,则有θn=θn-1+△θ，由此推导出θn=θ0+n△θ，由于计算周期是
Tpwm，所以在Topen时间内，就有n=Topen/Tpwn,同时初始值θ0=0，所以有
θn=Topen/Tpwn*△θ,又因为θn=ωn*Tpwn，所以得出△θ的值定义如下
*/
#define DELTA_STARTUP_RAMP	(unsigned int)(MINSPEEDINRPM*POLEPAIRS*LOOPTIMEINSEC* \
							LOOPTIMEINSEC*65536*65536/(60*OPENLOOPTIMEINSEC))

// Number of control loops that must execute before the button routine is executed.
#define	BUTPOLLOOPCNT	(unsigned int)(BUTPOLLOOPTIME/LOOPTIMEINSEC)


//----math parameter:
#define	UPOWER				4096//(24*16384)						//----power voltage: 1v=2^14=16384  dec
#define UoutMax				2896//(278045)						//----Uout's Maximum：UPOWER/sqrt(2)
#define Ux					3344//(321059)						//----Ux voltage：UPOWER*(sqrt(2)/sqrt(3))
#define Sq3Ux				5791//(556209)						//----sqrt(3)*Ux
//----pwm parameter:

#define PWM_DEAD_RISE_CYCLE	88								//----dead_rise=2us
#define PWM_DEAD_FALL_CYCLE	88								//----dead_fall=2us

#define PWM_UOUT			2687							//((UoutMax*(PWM_CLOCK_CYCLE-PWM_DEAD_RISE_CYCLE))/PWM_CLOCK_CYCLE)

#define	lock0	0//0deg	2913-1deg
#define	lock60  10922//60deg
#define	lock120 21845//120deg
#define	lock180 32767//180deg
#define	lock240 -21844//240deg
#define	lock300 -10922//300deg
#define	lock360 -1//360deg

#define DUTY_PID_P 250
#define DUTY_PID_I 50
#define DUTY_PID_D 0

#define UOUTMIN 800

#define UBUS_TIMES  1000    //母线稳定时间

#endif

