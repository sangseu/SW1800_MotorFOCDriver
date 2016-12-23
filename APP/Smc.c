/***********************************************************************
 *                                                                     *
 *    Filename:       smcpos.c                                         *
 *    Date:           13/04/2015                                         *
 *                                                                     *
 *                                                                     *
 ***********************************************************************
 *      Code Description                                               *
 *                                                                     *
 *  This file implements a slide mode observer. This observer is used  *
 *  to estimate rotor position and speed. Rotor position, Theta, is    *
 *  then compensated from phase delays introduced by the filters       *
 *                                                                     *
 **********************************************************************/

#include "general.h"
#include "smc.h"
#include "SWM1800.h"

/*****************atan2CORDIC函数****************
256表示1度，精确到1/256 = 0.00390625度
45度表示45*256 = 11520
建立一个表格如下:(度数*256)
tan(45)= 1  
tan(26.565051177078)= 1/2  
tan(14.0362434679265)= 1/4  
tan(7.1250163489018)= 1/8  
tan(3.57633437499735)= 1/16  
tan(1.78991060824607)= 1/32  
tan(0.8951737102111)= 1/64  
tan(0.4476141708606)= 1/128  
tan(0.2238105003685)= 1/256 
tan(0.1119056770662)= 1/512 
tan(0.0559528918938)= 1/1024
tan(0.027976452617)=1/2048
tan(0.01398822714227) = 1/4096
tan(0.006994113675353)=1/8192
tan(0.003497056850704)=1/16384

45.0*256=11520
26.5650511770078*256 = 6800.65310133196取整6801
14.0362434679265*256 = 3593.27832778918取整3593 
 如此类推计算出来的表格
 arctg(y/x)
**************************************************/
/**
*针对电机应用，PI(180度)对应的数值为32767，
*那么每一度对应的数值为32767/180 约等于182
*因此对下表的角度数值*182得到三角数值
tan(45)= 1  
tan(26.565051177078)= 1/2  
tan(14.0362434679265)= 1/4  
tan(7.1250163489018)= 1/8  
tan(3.57633437499735)= 1/16  
tan(1.78991060824607)= 1/32  
tan(0.8951737102111)= 1/64  
tan(0.4476141708606)= 1/128  
tan(0.2238105003685)= 1/256 
tan(0.1119056770662)= 1/512 
tan(0.0559528918938)= 1/1024
tan(0.027976452617)=1/2048
tan(0.01398822714227) = 1/4096
tan(0.006994113675353)=1/8192
tan(0.003497056850704)=1/16384

45*32767/180=8192
26.565051177078*32767/180=4836
..........
得到数组angle[]={8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 3, 1, 1,0}; 

**/
#define NEG_PI_BY_2  0xC000
#define PI_BY_2		0x3FFF
#define PI			0x7FFF
#define NEG_PI		0x8000

#if 0
static __inline s16 atan2CORDIC(s16 x,s16 y)//求actan(x/y)的值
{
	 const s16 angle[] = {8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 3, 1, 1,0};   
	//     const unsigned int angle[] = {11520, 6801, 3593, 1824, 916, 458, 229, 115, 57, 29, 14, 7, 4, 2, 1};
	u8 i = 0;  
	s16 x_new, y_new, ytemp = 0, xtemp = 0;  
	s16 angleSum = 0; 

    if( x<10 && x>-10 && y<10 && y>-10 )
     {
          x *= 1024;
          y *= 1024;  
     }

	if(x == Q15(-1))
		return (angleSum = NEG_PI_BY_2);
	else 
	{
		if(y == Q15(-1))
			return (angleSum = PI);
		else
		{
			if(y < 0)
			{
				ytemp = y;	//ytemp=TEMP y
				if(x > 0)		//tg(PI/2 + X)=-ctg(x)
				{
					y = x;
					x = -ytemp;
					angleSum = PI_BY_2;
				}
				else
				{
					y = -x;	//tg(-PI/2+x) = -ctg(x)
					x = ytemp;
					angleSum = NEG_PI_BY_2;					
				}
			}
			xtemp = x >> 1;
			ytemp = y >> 1;		
			 for(i = 0; i < 10; i++)  
			 {
				if(xtemp >= 0)  
				{  
					y_new = ytemp + (xtemp >> i);
					x_new = xtemp - (ytemp >> i);			
					ytemp = y_new;
					xtemp = x_new;
					angleSum += angle[i];
				}  
				else  
				{  
					y_new = ytemp - (xtemp >> i);
					x_new = xtemp + (ytemp >> i);						
					ytemp = y_new;
					xtemp = x_new;
					angleSum -= angle[i];  
				}
			}				
		}
        return angleSum;
	}
}
#endif

#if 0
static __inline int atan2CORDIC(s16 x,s16 y)
{
	 const short angle[] = {8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 3, 1, 1,0};   
     s16 i = 0;  
     s16 x_new, y_new;  
     s16 angleSum = 0; 
     s16 quar=0;

     if( x>0 && y>0 )
     {
          quar=1;
     }
     else if( x<0 && y>0 )
     {
          quar=2;
          x = -x;
          y = -y;
     }
     else if( x<0 && y<0 )
     {
          quar=3;
          x = -x;
          y = -y;
     }
     else if( x>0 && y<0 )
     {
          quar=4;
     }
     else
     {
          if( x==0 )
            {
                    if( y>0 )
                        return 16384;
                    else if( y<0 )
                        return -16384;
                    else
                        return 0;
            }
            else if( y==0 )
            {
                    if( x>0 )
                        return 0;
                    else if( x<0 )
                        return -32768;
                    else
                        return 0;
            }
            else 
                return 0;
     }


     for(i = 0; i < 15; i++)  
     {
            if(y > 0)  
            {  
                    x_new = x + (y >> i);  
                    y_new = y - (x >> i);  
                    x = x_new;  
                    y = y_new;  
                    angleSum += angle[i];  
            }  
            else  
            {  
                    x_new = x - (y >> i);  
                    y_new = y + (x >> i);  
                    x = x_new;  
                    y = y_new;  
                    angleSum -= angle[i];  
            }  
    }

    if( quar == 1 || quar== 4 )
    {
        angleSum = angleSum;  
    }
    else if( quar ==2 )
    {
        angleSum = 32767 + angleSum;  
    }
    else if( quar ==3 )
    {
        angleSum = -32768 + angleSum;
    }

    return angleSum;
}
#endif

static __inline s16 atan2CORDIC(s16 x,s16 y)
{
	s16 angleSum = 0; 
	u16 x_new, y_new;  
	s16 quar=0;
	u32 quot;

	if( x>0 && y>0 )
	{
		quar=1;
		x_new = x;
		y_new = y;
	}
	else if( x<0 && y>0 )
	{
		quar=2;
		x_new = -x;
		y_new = y;
	}
	else if( x<0 && y<0 )
	{
		quar=3;
		x_new = -x;
		y_new = -y;
	}
	else if( x>0 && y<0 )
	{
		quar=4;
		x_new = x;
		y_new = -y;
	}
	else
	{
		if( x==0 )
		{
			if( y>0 )
				return 16384;
			else if( y<0 )
				return -16384;
			else
				return 0;
		}
		else if( y==0 )
		{
			if( x>0 )
				return 0;
			else if( x<0 )
				return -32768;
			else
				return 0;
		}
		else 
			return 0;
	}

	DIV_Div(y_new*16384, x_new);    //cal Y/X
	while(DIV_Div_IsBusy());        //wait for division finish
	quot = DIV->QUO;
	if( quot <= 820 )
		quot = 820;
	else if( quot > (10000*16384 - 1) )
		quot = 10000*16384 - 1;
    
	CORDIC_Arctan(quot);        //atan(Y/X)
	while(CORDIC_Arctan_IsDone() == 0);      //wait for atan finish

	DIV_Div(CORDIC_Arctan_Result()*2*10000, 31416);   //atan()/16384/3.1415926*32767  //将弧度转换为角度,0~PI对应0~32767
	while(DIV_Div_IsBusy());        //wait for division finish
    
	angleSum = DIV->QUO;    //计算出0~16384角度

	if( quar ==2 )
	{
		angleSum = 32767 - angleSum; 
	}
	else if( quar ==3 )
	{
		angleSum = -32768 + angleSum;
	}
	else if( quar ==4 )
	{
		angleSum = -angleSum;
	}
      
	return angleSum;
}



//s->EstIalpha 与s->EstIbeta已经是Q15格式
__inline void CalcEstI(SMC *s)
{
	s16 Ggain = s->Gsmopos;	//Q15格式
	s32 valtemp1,valtemp2;

	valtemp1 = Ggain * (s->Valpha - s->Ealpha - s->Zalpha) ;
	valtemp2 = s->Fsmopos * s->EstIalpha;
	s->EstIalpha = RIGHSHIFT15(valtemp1) + RIGHSHIFT15(valtemp2);//s->EstIalpha = (Ggain * (s->Valpha - s->Ealpha - s->Zalpha) + s->Fsmopos * s->EstIalpha)/DIV_RATIO;

	valtemp1 = Ggain * (s->Vbeta - s->Ebeta - s->Zbeta);
	valtemp2 = s->Fsmopos * s->EstIbeta;
	s->EstIbeta = RIGHSHIFT15(valtemp1) + RIGHSHIFT15(valtemp2);//s->EstIbeta = (Ggain * (s->Vbeta - s->Ebeta - s->Zbeta) + s->Fsmopos * s->EstIbeta)/DIV_RATIO;
	
}

__inline void CalcIError(SMC *s)
{
	s->IalphaError = s->EstIalpha - (s->Ialpha);	//Q15
	s->IbetaError = s->EstIbeta - (s->Ibeta);		//Q15
	//     s->IalphaError = -s->EstIalpha + (s->Ialpha);	//Q15
	// 	s->IbetaError = -s->EstIbeta + (s->Ibeta);		//Q15
}


__inline void CalcZalpha(SMC *s)
{
	s32 Di, Vi, Qi, Ri;

	Di = s->Kslide * s->IalphaError;
	Vi = s->MaxSMCError;

	DIV_Fun(Di, Vi, &Qi, &Ri);

	s->Zalpha = Qi;    //s->Zalpha = ((s->Kslide * s->IalphaError))/s->MaxSMCError;	//Q15格式
}

__inline void CalcZbeta(SMC *s)
{
	s32 Di, Vi, Qi, Ri;

	Di = s->Kslide * s->IbetaError;
	Vi = s->MaxSMCError; 

	DIV_Fun(Di, Vi, &Qi, &Ri);

	s->Zbeta = Qi;    //s->Zbeta = ((s->Kslide * s->IbetaError))/s->MaxSMCError;	//Q15
}


__inline void CalcBEMF(SMC *s)
{
	s32 valtemp;
	s16 KslfT = s->Kslf;	//Q15
	s16 KslfFinalT = s->KslfFinal;//Q15

	valtemp = KslfT * (s->Zalpha - s->Ealpha); 
	s->Ealpha += RIGHSHIFT15(valtemp);//s->Ealpha = s->Ealpha + RIGHSHIFT15(valtemp);//s->Ealpha = s->Ealpha + ((KslfT * (s->Zalpha - s->Ealpha))/DIV_RATIO);

	valtemp = KslfT * (s->Zbeta - s->Ebeta);
	s->Ebeta += RIGHSHIFT15(valtemp);//s->Ebeta = s->Ebeta + RIGHSHIFT15(valtemp);//s->Ebeta = s->Ebeta + ((KslfT * (s->Zbeta - s->Ebeta))/DIV_RATIO);

	valtemp = KslfFinalT * (s->Ealpha - s->EalphaFinal);
	s->EalphaFinal += RIGHSHIFT15(valtemp);//s->EalphaFinal = s->EalphaFinal + RIGHSHIFT15(valtemp);//s->EalphaFinal = s->EalphaFinal + ((KslfFinalT * (s->Ealpha -s->EalphaFinal))/DIV_RATIO);

	valtemp = KslfFinalT * (s->Ebeta - s->EbetaFinal);
	s->EbetaFinal += RIGHSHIFT15(valtemp);//s->EbetaFinal = s->EbetaFinal + RIGHSHIFT15(valtemp);//s->EbetaFinal = s->EbetaFinal + ((KslfFinalT * (s->Ebeta - s->EbetaFinal))/DIV_RATIO);
}

__inline void CalcOmegaFltred(SMC *s)
{
	s32 valtemp;
	s16 FiltOmCoefT = s->FiltOmCoef ;//Q15

	valtemp = FiltOmCoefT * (s->Omega - s->OmegaFltred); 
	s->OmegaFltred += RIGHSHIFT15(valtemp);//Q15// 	s->OmegaFltred = s->OmegaFltred + ((FiltOmCoefT * (s->Omega - s->OmegaFltred))/DIV_RATIO);//Q15
}

s32 tempv;
s32 start=0;
s32 IndPre=0;
s32 dex;
s32 temptheta=0;
extern s32 AdIndexA;
extern s16 Kself_omega0;
extern s16 Run_nomal;
long PreOmega = 0;
extern u16 Loopflag;
extern s16 delta_angle;
extern u32 timecout,rem;
s16 kseltemp;

void SMC_Position_Estimation (SMC *s)
{
	s32 Di, Vi, Qi, Ri;

	CalcEstI(s);

	CalcIError(s);

/* ***********Sliding control calculator********************/
/*******************
滑模控制也叫变结构控制，
将被控量往稳定却换面上逼近。
********************/

	if ( (s->IalphaError <= s->MaxSMCError) && (s->IalphaError >= (-s->MaxSMCError)) )
	{
		// s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zalpha will be proportional to the
		// error (Ialpha - EstIalpha) and slide mode gain, Kslide.
		CalcZalpha(s);
	}
	else if ((s->IalphaError) > 0)
	{
		s->Zalpha = s->Kslide;
	}
	else
	{
		s->Zalpha = -s->Kslide;
	}

	if ( (s->IbetaError <= s->MaxSMCError) && (s->IbetaError >= (-s->MaxSMCError)) )
	{
		// s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zbeta will be proportional to the
		// error (Ibeta - EstIbeta) and slide mode gain, Kslide.
		CalcZbeta(s);
	}
	else if ((s->IbetaError) > 0)
		s->Zbeta = s->Kslide;
	else
		s->Zbeta = -s->Kslide;
/************Sliding control calculator end**********************/
	
/*************************************************
对滑模控制器的输出一阶低通滤波得
到反电动势，由于一阶滤波后输出的
反电动势波动比较大，影响角度的计
算，因此，用于计算角度的电压矢量
再进行一次低通滤波。原理可以参考
电路的RC低通滤波电路。
s->Ealpha = s->Ealpha + s->Kslf * s->Zalpha -s->Kslf * s->Ealpha
s->Ebeta = s->Ebeta + s->Kslf * s->Zbeta -s->Kslf * s->Ebeta
s->EalphaFinal = s->EalphaFinal + s->KslfFinal * s->Ealpha- s->KslfFinal * s->EalphaFinal
s->EbetaFinal = s->EbetaFinal + s->KslfFinal * s->Ebeta- s->KslfFinal * s->EbetaFinal
*************************************************/
	CalcBEMF(s);

	// Rotor angle calculator -> Theta = atan(-EalphaFinal,EbetaFinal)

	s->Theta = atan2CORDIC(s->EbetaFinal, -s->EalphaFinal);//Q15

	temptheta = PrevTheta - s->Theta;//temptheta = s->Theta - PrevTheta;//
	
	/*转换成到(-32768,32767)这个量化角度范围，也就是(-PI,PI)*/
	if( temptheta < -32768 )
		temptheta += 65536;
	else if( temptheta > 32767 )
		temptheta -= 65536;


	AccumTheta += temptheta;//s->Theta - PrevTheta;//
	PrevTheta = s->Theta;

	AccumThetaCnt++;

	if (AccumThetaCnt == IRP_PERCALC)
	{

		s->Omega = -AccumTheta;

		AccumThetaCnt = 0;
		AccumTheta = 0;

	}
	//                    Q15(Omega) * 60
	// Speed RPMs = -----------------------------
	//               SpeedLoopTime * Motor Poles
	// For example:
	//    Omega = 0.5
	//    SpeedLoopTime = 0.001
	//    Motor Poles (pole pairs * 2) = 10
	// Then:
	//    Speed in RPMs is 3,000 RPMs

	// s->OmegaFltred = s->OmegaFltred + FilterCoeff * s->Omega
	//								   - FilterCoeff * s->OmegaFltred
	
/*******角度一阶滤波后来计算截止频率*********/
	CalcOmegaFltred(s);

	// Adaptive filter coefficients calculation
	// Cutoff frequency is defined as 2*_PI*electrical RPS
	//
	// 		Wc = 2*_PI*Fc.
	// 		Kslf = Tpwm*2*_PI*Fc
	//
	// Fc is the cutoff frequency of our filter. We want the cutoff frequency
	// be the frequency of the drive currents and voltages of the motor, which
	// is the electrical revolutions per second, or eRPS.
	//
	// 		Fc = eRPS = RPM * Pole Pairs / 60
	//
	// Kslf is then calculated based on user parameters as follows:
	// First of all, we have the following equation for RPMS:
	//
	// 		RPM = (Q15(Omega) * 60) / (SpeedLoopTime * Motor Poles)
	//		Let us use: Motor Poles = Pole Pairs * 2
	//		eRPS = RPM * Pole Pairs / 60), or
	//		eRPS = (Q15(Omega) * 60 * Pole Pairs) / (SpeedLoopTime * Pole Pairs * 2 * 60)
	//	Simplifying eRPS
	//		eRPS = Q15(Omega) / (SpeedLoopTime * 2)
	//	Using this equation to calculate Kslf
	//		Kslf = Tpwm*2*_PI*Q15(Omega) / (SpeedLoopTime * 2)
	//	Using diIrpPerCalc = SpeedLoopTime / Tpwm
	//		Kslf = Tpwm*2*Q15(Omega)*_PI / (diIrpPerCalc * Tpwm * 2)
	//	Simplifying:
	//		Kslf = Q15(Omega)*_PI/diIrpPerCalc
	//
	// We use a second filter to get a cleaner signal, with the same coefficient
	//
	// 		Kslf = KslfFinal = Q15(Omega)*_PI/diIrpPerCalc
	//
	// What this allows us at the end is a fixed phase delay for theta compensation
	// in all speed range, since cutoff frequency is changing as the motor speeds up.
	// 
	// Phase delay: Since cutoff frequency is the same as the input frequency, we can
	// define phase delay as being constant of -45 DEG per filter. This is because
	// the equation to calculate phase shift of this low pass filter is 
	// arctan(Fin/Fc), and Fin/Fc = 1 since they are equal, hence arctan(1) = 45 DEG.
	// A total of -90 DEG after the two filters implemented (Kslf and KslfFinal).

/***计算截止频率，用于反电动势的计算****/
	Di = s->OmegaFltred ;//
	Vi = IRP_PERCALC  ;//
	DIV_Fun(Di, Vi, &Qi, &Ri);
	
	s->Kslf = Qi; 
	s->KslfFinal = s->FiltOmCoef =  s->Kslf;
/***********END***************/

	// Since filter coefficients are dynamic, we need to make sure we have a minimum
	// so we define the lowest operation speed as the lowest filter coefficient

	if( s->Kslf < Kself_omega0 )//if (s->Kslf < Q15(OMEGA0 * _PI / IRP_PERCALC)) //
	{
		s->Kslf = s->KslfFinal = s->FiltOmCoef = Kself_omega0;//s->Kslf = s->KslfFinal = Q15(OMEGA0 * _PI / IRP_PERCALC);//

	}
	s->Theta +=  s->ThetaOffset;

	return;
}

void SMCInit(SMC *s)
{
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //            Ts
    // Gsmopos = ----
    //            L
    // Ts = Sampling Period. If sampling at PWM, Ts = 50 us
    // R = Phase Resistance. If not provided by motor datasheet,
    //     measure phase to phase resistance with multimeter, and
    //     divide over two to get phase resistance. If 4 Ohms are
    //     measured from phase to phase, then R = 2 Ohms
    // L = Phase inductance. If not provided by motor datasheet,
    //     measure phase to phase inductance with multimeter, and
    //     divide over two to get phase inductance. If 2 mH are
    //     measured from phase to phase, then L = 1 mH
//F与G增益取值范围为(0,1]????
	if (Q15(PHASERES * LOOPTIMEINSEC) > Q15( PHASEIND))
		s->Fsmopos = Q15(0.0);
	else
		s->Fsmopos = Q15(1) - Q15(PHASERES * LOOPTIMEINSEC / PHASEIND);

	if (Q15(LOOPTIMEINSEC) > Q15(PHASEIND))
		s->Gsmopos = Q15(0.99999);
	else
		s->Gsmopos = Q15(LOOPTIMEINSEC / PHASEIND);

	s->Kslide = Q15(SMCGAIN);
	s->MaxSMCError = Q15(MAXLINEARSMC);

	s->FiltOmCoef = 65536 * OMEGA_CUF / IRP_PERCALC;// Cutoff frequency for omega filter
					
	Kself_omega0 = 65536 * OMEGA_CUF / IRP_PERCALC;//
	s->ThetaOffset = CONSTANT_PHASE_SHIFT;
	return;
}

