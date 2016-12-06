#include "typedef.h"

#include "General.h"
#include "Parms.h"
#include "SVGen.h"
#include "ReadADC.h"
#include "MeasCurr.h"
#include "Control.h"
#include "PI.h"
#include "Park.h"
#include "UserParms.h"
#include "Smc.h"
#include "FdWeak.h"
#include "faultprotect.h"

#include "SWM1800.h"

#define calculate_frequency
//#undef calculate_frequency 
#define use_cordic_module

#define PinButton1	!((GPIOC->DATA >> PIN2) & 0x01)	//Low Level is press 

/********************* Variables to Calculate control board frequency *********************************/
u32 fg_outfrequence;
typedef struct
{
	u32 InputFreq;      //输入频率
	u8 CountStartFlag;  //开始计数标志,0=未开始，1=开始, 2=停止
	u32 InputFreqcnt;   //输入脉冲计数值
	u32 Timercnt;       //定时器计数值
}InpFreq;
InpFreq InpFreqCnt;

void CountInputFreq_Init();
void calculate_rotate_speed(InpFreq *pfreq);

MOTOR_FAULT motor_fault;

PHASE_CURRENT Phase_current_detect;

/********************* Variables to Calculate control board frequency *********************************/
int Duty0,Duty1,Duty2;
s16 Sector;
s16 PWM_CLOCK_CYCLE = LOOPINTCY/2;				
s16 LoopFlag;

int uout = 0;

SMC smc1 = SMC_DEFAULTS;

unsigned long Startup_Ramp = 0;	/* Start up ramp in open loop. This variable
								is incremented in CalculateParkAngle()
								subroutine, and it is assigned to 
								ParkParm.qAngle as follows:
								ParkParm.qAngle += (int)(Startup_Ramp >> 16);*/

u32 Startup_Lock = 0;	/* This is a counter that is incremented in
								CalculateParkAngle() every time it is called. 
								Once this counter has a value of LOCK_TIME, 
								then theta will start increasing moving the 
								motor in open loop. */

union{
	struct
	{
		unsigned OpenLoop:1;		// Indicates if motor is running in open or closed loop
		unsigned RunMotor:1;		// If motor is running, or stopped.
		unsigned EnTorqueMod:1;	// This bit enables Torque mode when running closed loop
		unsigned EnVoltRipCo:1;	// Bit that enables Voltage Ripple Compensation
		unsigned Btn1Pressed:1;	// Button 1 has been pressed.
		unsigned Btn2Pressed:1;	// Button 2 has been pressed.
		unsigned ChangeMode:1;	// This flag indicates that a transition from open to closed
				// loop, or closed to open loop has happened. This
				// causes DoControl subroutine to initialize some variables
				// before executing open or closed loop for the first time
		unsigned ChangeSpeed:1;	// This flag indicates a step command in speed reference.
				// This is mainly used to analyze step response
		unsigned    :8;
	}bit;
	unsigned int Word;
} uGF;

tPIParm 	PIParmD;	// Structure definition for Flux component of current, or Id
tPIParm 	PIParmQ;	// Structure definition for Torque component of current, or Iq
tPIParm 	PIParmW;	// Structure definition for Speed, or Omega
tReadADCParm ReadADCParm;	// Struct used to read ADC values.

tMeasCurrParm MeasCurrParm;

tMotorParm MotorParm;

tParkParm ParkParm={-16384,0,0}; 

tSVGenParm SVGenParm;


// Speed Calculation Variables

u32 iADCisrCnt = 0;	// This Counter is used as a timeout for polling the push buttons
						// in main() subroutine. It will be reset to zero when it matches
						// dButPolLoopCnt defined in UserParms.h
s16 PrevTheta = 0;	// Previous theta which is then substracted from Theta to get
						// delta theta. This delta will be accumulated in AccumTheta, and
						// after a number of accumulations Omega is calculated.
s16 AccumTheta = 0; // Accumulates delta theta over a number of times
u32 AccumThetaCnt = 0; // Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is 
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.

// Vd and Vq vector limitation variables
u32 qVdSquared = 0; // This variable is used to know what is left from the VqVd vector
					// in order to have maximum output PWM without saturation. This is
					// done before executing Iq control loop at the end of DoControl()

s16 DCbus = 0;		// DC Bus measured continuously and stored in this variable
					// while motor is running. Will be compared with TargetDCbus
					// and Vd and Vq will be compensated depending on difference
					// between DCbus and TargetDCbus

s16 TargetDCbus = 0;// DC Bus is measured before running motor and stored in this
					// variable. Any variation on DC bus will be compared to this value
					// and compensated linearly.	

s16 Theta_error = 0;// This value is used to transition from open loop to closed looop. 
					// At the end of open loop ramp, there is a difference between 
					// forced angle and estimated angle. This difference is stored in 
					// Theta_error, and added to estimated theta (smc1.Theta) so the 
					// effective angle used for commutating the motor is the same at 
					// the end of open loop, and at the begining of closed loop. 
					// This Theta_error is then substracted from estimated theta 
					// gradually in increments of 0.05 degrees until the error is less
					// than 0.05 degrees.

s16 ADIa = 0;		//AD sample Ia value
s16 ADIb = 0;		//AD sample Ib value

void UartInit(void)
{	
	UART_InitStructure UART_initStruct;

	PORT_Init(PORTA, PIN8, FUNMUX_UART0_RXD, 1);	//GPIOA.0配置为UART0输入引脚
	PORT_Init(PORTA, PIN7, FUNMUX_UART0_TXD, 0);	//GPIOA.1配置为UART0输出引脚

	UART_initStruct.Baudrate = 128000;//230400;//115200;
	UART_initStruct.RXThreshold = 1;
	UART_initStruct.RXThresholdIEn = 0;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutIEn = 0;
	UART_Init(UART0, &UART_initStruct);

	IRQ_Connect(IRQ0_15_UART0, IRQ2_IRQ, 2);  

	UART_Open(UART0);	
}

#define	putstring2(x) uart_putstring2((x))

void _CRC16(char *Array, char *Rcvbuf, unsigned int Len) 
{
	uint16_t  IX, IY;
	uint16_t  CRC_val;
	CRC_val = 0xFFFF;//set all 1
	if (Len<=0)
	CRC_val = 0;
	else
	{
		Len--;
		for (IX=0;IX<=Len;IX++) 
		{ 
			CRC_val=CRC_val^(uint16_t)(Array[IX]); 
			for(IY=0;IY<=7;IY++) 
			{
				if ((CRC_val&1)!=0) CRC_val=(CRC_val>>1)^0xA001; 
				else CRC_val=CRC_val>>1;    // 
			}
		} 
	} 
	Rcvbuf[0] = (CRC_val & 0xff00)>>8;  //??? 
	Rcvbuf[1] = (CRC_val & 0x00ff);     //??? 
}  

void uart_putstring2(char buf[])
{
	char num=0;
	for(num=0;num<10;num++)
	{
		UART_WriteByte(UART0,buf[num]);
		while(UART_IsTXFIFOFull(UART0));	//等待发送完成
	} 	
}

void Communication(int16_t data0,int16_t data1,int16_t data2, int16_t data3)
{
	char pact[10]={0};
	char crc[2];

	pact[0] = data0 & 0xff;
	pact[1] = data0>>8;
	pact[2] = data1 & 0xff;
	pact[3] = data1>>8;
	pact[4] = data2 & 0xff;
	pact[5] = data2>>8;
	pact[6] = data3 & 0xff;
	pact[7] = data3>>8;
	_CRC16(pact, crc, 8);
	pact[8] = crc[1];
	pact[9] = crc[0];
	putstring2(pact);	
}


extern s32 Ta,Tb,Tc;
extern s16 Pwm0A,Pwm1A,Pwm2A;;
extern s32 t1temp,t2temp;
extern s16 CorrADC0,CorrADC1;
extern s32 tempv;

extern s16 temptheta;

u16 Loopflag=1;
s16 Kself_omega0;
extern s16 Qref;
s32 SpeedV;
s32 SpeedTotal;
s32 Speedtimes;
s32 VdcTotal,Vdctimes;
s16 Run_nomal = 1;
s16 delta_angle = 150;
u32 lost_times;
s16 T0_t = 2650;
s16 PWM_per = 5300;
s16 SPEED;
volatile s16 Speed_REF =500,Speed_REF1,Speed_REF_old=0,W_REF=8000;
s16 speedtemp;
s16 count;
s16 cntt;
s16 View1,View2,View3,View4;
u32 timecout,rem;
s16 sector;
s16 stas = 0;
s16 reset_delaytimes;
volatile s16 VDC_status = 0, VDC_status_tiems =0;        //直流母线电压状态
volatile u16 SPREF=300;
s16 View_Variable1,View_Variable2,View_Variable3,View_Variable4;
int main(void)
{	
	SystemInit();
//#ifndef use_cordic_module
	DIV_Init(DIV);			//div初始化过程不需要
//#endif
	CORDIC_Init(CORDIC);	//cordic初始化时已经包含有div初始化过程

	SMCInit(&smc1);
	SetupPorts();
	SetupControlParameters(); 
	FWInit();
#ifdef calculate_frequency
	CountInputFreq_Init();
//#else
	UartInit();
#endif
	uGF.Word = 0;                   // clear flags

#ifdef TORQUEMODE		//转矩模式
	uGF.bit.EnTorqueMod = 1;
#endif

#ifdef ENVOLTRIPPLE	//母线电压补偿
	uGF.bit.EnVoltRipCo = 1;
#endif

	uGF.bit.OpenLoop = 1;			// start in openloop
	uGF.bit.RunMotor=0;
	PIParmD.qdSum = 0;
	PIParmQ.qdSum = 0;
	PIParmW.qdSum = 0;
	
	motor_fault_init(&motor_fault);
	
	SetupParm(); 
		
	// Run the motor
	while(1)
	{
		View1 = View_Variable1;//PIParmQ.qInRef;//ParkParm.qIa;//PIParmW.qInMeas;//Pwm0A;//smc1.Ialpha;//ParkParm.qValpha;//
		View2 = View_Variable2;//smc1.OmegaFltred;//PIParmD.qInRef;//ParkParm.qIb;//PIParmW.qInRef;//Pwm1A;//smc1.EstIalpha;//ParkParm.qVq;//ParkParm.qVbeta;//
		View3 = View_Variable3;//PIParmW.qInRef;//ParkParm.qAngle;//smc1.Theta;//PIParmD.qInRef;//PIParmQ.qInMeas;//PIParmW.qOut;//smc1.Kslf;//ParkParm.qIa;//SPEED;//ParkParm.qIq;//PIParmD.qOut;//Pwm2A;//smc1.Zalpha;//stas*10000;//
		View4 = View_Variable4;//smc1.Ebeta;//SPREF;//PIParmQ.qOut;////0;//ParkParm.qIb;//TargetDCbus;//smc1.IalphaError;//CtrlParm.qVqRef;//sector;//

		Communication(View1, View2, View3, View4);

	//		SPEED = smc1.OmegaFltred*60/(65535*POLEPAIRS*SPEEDLOOPTIME);

		motor_fault_detect(&motor_fault);
		calculate_rotate_speed(&InpFreqCnt);
        
        motor_fault.Led_indicate->Systemstatus = motor_fault.fault_code;
        
 //       SPREF =600;
        
                
		if(SPREF >360)
		{				
			if( SPREF > 950 )
                SPREF = 950;
			W_REF = SPREF*65535*POLEPAIRS*SPEEDLOOPTIME/60;//700*65535*POLEPAIRS*SPEEDLOOPTIME/60;

			if(!uGF.bit.RunMotor)
			{
				PWMG->CHEN |= 0x7f;	
				// executed for the first time
				uGF.bit.ChangeMode = 1;	// Ensure variable initialization when open loop is
				uGF.bit.RunMotor = 1;               //then start motor
			}
		}
		else
		{
			PWMG->CHEN &= ~0x7f;	//pwm0a,0b,pwm1a,1b,pwm2a,2b,pwm3a off
			
			uGF.bit.OpenLoop = 1;	
			uGF.bit.RunMotor=0;

			Startup_Lock = 0;
			Startup_Ramp = 0;
		
			SetupControlParameters(); 
			PIParmD.qdSum = 0;
			PIParmQ.qdSum = 0;
			PIParmW.qdSum = 0;
			ParkParm.qAngle = -16384;			
		}
			

	}   // End of Run Motor loop
}

//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,
extern s32 SpeedV;

s16 Qref  = REFINAMPS(0.5),Dref=0;;
s16 AccumCurrentCnt=0;      //Current loop cal times
void DoControl( void )
{
	s32 Q_err,D_err;

	if( uGF.bit.OpenLoop )
	{
		// OPENLOOP:	force rotating angle, and control Iq and Id
		//				Also limits Vs vector to ensure maximum PWM duty
		//				cycle and no saturation

		// This If statement is executed only the first time we enter open loop,
		// everytime we run the motor
		if( uGF.bit.ChangeMode )
		{
			// just changed to openloop
			uGF.bit.ChangeMode = 0;
			// synchronize angles

			// VqRef & VdRef not used
			CtrlParm.qVqRef = 0;
			CtrlParm.qVdRef = 0;
			CtrlParm.qVelRef = 0;
			//			Startup_Lock = 0;
			Startup_Ramp = 0;
			// Initialize SMC
			smc1.Valpha = 0;
			smc1.Ealpha = 0;
			smc1.EalphaFinal = 0;
			smc1.Zalpha = 0;
			smc1.EstIalpha = 0;
			smc1.Vbeta = 0;
			smc1.Ebeta = 0;
			smc1.EbetaFinal = 0;
			smc1.Zbeta = 0;
			smc1.EstIbeta = 0;
			smc1.Ialpha = 0;
			smc1.IalphaError = 0;
			smc1.Ibeta = 0;
			smc1.IbetaError = 0;
			smc1.Theta = 0;
			smc1.Omega = 0;
		}
		// Enter initial torque demand in Amps using REFINAMPS() macro.
		// Maximum Value for reference is defined by shunt resistor value and 
		// differential amplifier gain. Use this equation to calculate 
		// maximum torque in Amperes:
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
		// If motor requires more torque than Maximum torque to startup, user
		// needs to change either shunt resistors installed on the board,
		// or differential amplifier gain.

		CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);//3971;//Refinamps;//REFINAMPS(TORQ);//	//Q15
		if(AccumThetaCnt == 0)//??????
		{
			PIParmW.qInMeas = smc1.Omega;	//Q15,???????????
		}									//???????????

		// PI control for D		
		PIParmD.qInMeas = ParkParm.qId;	//Q15
		PIParmD.qInRef  = CtrlParm.qVdRef;//speed0;//	//Q15

		CalcPI(&PIParmD); //???PID
		ParkParm.qVd    = PIParmD.qOut;//-3440;//
		// Vector limitation
		// Vd is not limited
		// Vq is limited so the vector Vs is less than a maximum of 95%.
		// The 5% left is needed to be able to measure current through
		// shunt resistors.
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95
		// Vq = SQRT(0.95^2 - Vd^2)
		qVdSquared = (ParkParm.qVd * ParkParm.qVd);	//Q15
		PIParmQ.qOutMax = sqrt_16(968992850 - qVdSquared);
		PIParmQ.qOutMin = 0-PIParmQ.qOutMax;    //Qoutmin;//

		// PI control for Q
		PIParmQ.qInMeas = ParkParm.qIq;
		PIParmQ.qInRef  = CtrlParm.qVqRef;//speed0;//

		CalcPI(&PIParmQ);
		ParkParm.qVq = PIParmQ.qOut;//5500;//2300;//

		if (Startup_Lock < MotorParm.LockTime)
		{
			ParkParm.qVd = 0;
		}
	}

	else            // Closed Loop Vector Control
	{        
	// When it first transition from open to closed loop, this If statement is
	// executed
		if( uGF.bit.ChangeMode )
		{
			// just changed from openloop
			uGF.bit.ChangeMode = 0;
			// An initial value is set for the speed controller accumulation.
			//
			// The first time the speed controller is executed, we want the output
			// to be the same as it was the last time open loop was executed. So,
			// last time open loop was executed, torque refefernce was constant,
			// and set to CtrlParm.qVqRef.
			//
			// First time in closed loop, CtrlParm.qVqRef = PIParmW.qdSum >> 16
			// assuming the error is zero at time zero. This is why we set 
			// PIParmW.qdSum = (long)CtrlParm.qVqRef << 16.
			PIParmW.qdSum = (long)CtrlParm.qVqRef;//(long)CtrlParm.qVqRef << 16;
			PIParmW.qOut = CtrlParm.qVqRef;
			//			Startup_Lock = 0;
			CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);
			// 			//velocity reference ramp begins at minimum speed
			// 			CtrlParm.qVelRef = Q15(OMEGA0);

//			TIMR_Start(TIMR2);
            

		}  

	// Check to see if new velocity information is available by comparing
	// the number of interrupts per velocity calculation against the
	// number of velocity count samples taken.  If new velocity info
	// is available, calculate the new velocity value and execute
	// the speed control loop.

		if(AccumThetaCnt == 0)
		{
			// Execute the velocity control loop
			PIParmW.qInMeas = smc1.OmegaFltred;	
			PIParmW.qInRef  = W_REF;//CtrlParm.qVelRef;//

			CalcPI(&PIParmW);
		}

		++AccumCurrentCnt;
		if( AccumCurrentCnt >= IRP_CURRENT_PERCALC )
		{
			AccumCurrentCnt = 0;

			if (Theta_error > _0_05DEG || Theta_error < (0- _0_05DEG))
			{
				CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);
				PIParmW.qdSum = (long)CtrlParm.qVqRef;//(long)CtrlParm.qVqRef << 16;
				PIParmW.qOut = CtrlParm.qVqRef;
			}
			else
			{
				if (uGF.bit.EnTorqueMod)
				{
					CtrlParm.qVqRef = Qref;
				}
				else
				{
					CtrlParm.qVqRef = PIParmW.qOut;
				}
			}
		}

		// If the application is running in torque mode, the velocity
		// control loop is bypassed.  The velocity reference value, read
		// from the potentiometer, is used directly as the torque 
		// reference, VqRef. This feature is enabled automatically only if
		// #define TORQUEMODE is defined in UserParms.h. If this is not
		// defined, uGF.bit.EnTorqueMod bit can be set in debug mode to enable
		// torque mode as well.

		// 		if (uGF.bit.EnTorqueMod)
		// 			CtrlParm.qVqRef = CtrlParm.qVelRef;

		// Get Id reference from Field Weakening table. If Field weakening
		// is not needed or user does not want to enable this feature, 
		// let NOMINALSPEEDINRPM be equal to FIELDWEAKSPEEDRPM in
		// UserParms.h
		//        CtrlParm.qVdRef = FieldWeakening((CtrlParm.qVelRef ? CtrlParm.qVelRef:-CtrlParm.qVelRef));
		CtrlParm.qVdRef = FieldWeakening((W_REF ? W_REF:-W_REF));

		// PI control for D
		PIParmD.qInMeas = ParkParm.qId;
		PIParmD.qInRef  = 0;//CtrlParm.qVdRef;//////speed1;//

		CalcPI(&PIParmD);
		ParkParm.qVd = PIParmD.qOut;

		// If voltage ripple compensation flag is set, adjust the output
		// of the D controller depending on measured DC Bus voltage. This 
		// feature is enabled automatically only if #define ENVOLTRIPPLE is 
		// defined in UserParms.h. If this is not defined, uGF.bit.EnVoltRipCo
		// bit can be set in debug mode to enable voltage ripple compensation.
		//
		// NOTE:
		//
		// If Input power supply has switching frequency noise, for example if a
		// switch mode power supply is used, Voltage Ripple Compensation is not
		// recommended, since it will generate spikes on Vd and Vq, which can
		// potentially make the controllers unstable.

		// 		if(uGF.bit.EnVoltRipCo)
		// 			ParkParm.qVd = VoltRippleComp(PIParmD.qOut);
		// 		else

		//         PI_DTerm.err = PIParmD.qInMeas - PIParmD.qInRef;
		// 		ParkParm.qVd -= Cal_PID(&PI_DTerm);
		//         
		//         if( ParkParm.qVd > PIParmD.qOutMax)
		//             ParkParm.qVd = PIParmD.qOutMax;	//Q15
		//         else if( ParkParm.qVd < PIParmD.qOutMin)
		//             ParkParm.qVd = PIParmD.qOutMin;	//Q15


		// Vector limitation
		// Vd is not limited
		// Vq is limited so the vector Vs is less than a maximum of 95%. 
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95
		// Vq = SQRT(0.95^2 - Vd^2)

		qVdSquared = (ParkParm.qVd * ParkParm.qVd);
		PIParmQ.qOutMax = sqrt_16(968992850 - qVdSquared);
		PIParmQ.qOutMin = -PIParmQ.qOutMax;//1000; //Qoutmin;////8000;

		// PI control for Q
		PIParmQ.qInMeas = ParkParm.qIq;
		PIParmQ.qInRef  = CtrlParm.qVqRef;//Qref;//

		CalcPI(&PIParmQ);
		ParkParm.qVq = PIParmQ.qOut;


		// If voltage ripple compensation flag is set, adjust the output
		// of the Q controller depending on measured DC Bus voltage
		// 		if(uGF.bit.EnVoltRipCo)
		// 			ParkParm.qVq = VoltRippleComp(PIParmQ.qOut);
		// 		else

		//         PI_QTerm.err = PIParmQ.qInMeas - PIParmQ.qInRef;
		// 		ParkParm.qVq -= Cal_PID(&PI_QTerm);
		//         
		//         if( ParkParm.qVq > PIParmQ.qOutMax)
		//             ParkParm.qVq = PIParmQ.qOutMax;	//Q15
		//         else if( ParkParm.qVq < PIParmQ.qOutMin)
		//             ParkParm.qVq = PIParmQ.qOutMin;	//Q15

		// Limit, if motor is stalled, stop motor commutation
		if(smc1.OmegaFltred  < 30 )//&& (ParkParm.qIa>15000 || ParkParm.qIa<-15000) )
		{
		//             PWMG->CHEN &= ~0x3f;
		//   			uGF.bit.RunMotor = 0;
		// //             Run_nomal = 0;
		//             
		//             TIMR_Start(TIMR2);
		}

	//         timecout = (0xffffff - SysTick->VAL)/44;
	}
}		

//const u32 ADSinValue[375] = 
//{
//2048,2051,2054,2058,2061,2064,2068,2071,2074,2078,2081,2084,2087,2091,2094,2097,2100,2104,2107,2110,
//2113,2116,2120,2123,2126,2129,2132,2135,2138,2141,2144,2147,2150,2153,2155,2158,2161,2164,2166,2169,
//2172,2174,2177,2179,2182,2184,2187,2189,2192,2194,2196,2198,2201,2203,2205,2207,2209,2211,2213,2215,
//2216,2218,2220,2222,2223,2225,2226,2228,2229,2231,2232,2233,2234,2236,2237,2238,2239,2240,2241,2241,
//2242,2243,2244,2244,2245,2245,2246,2246,2247,2247,2247,2247,2247,2247,2247,2247,2247,2247,2247,2247,
//2246,2246,2246,2245,2245,2244,2243,2243,2242,2241,2240,2239,2238,2237,2236,2235,2234,2233,2231,2230,
//2228,2227,2226,2224,2222,2221,2219,2217,2215,2214,2212,2210,2208,2206,2204,2202,2199,2197,2195,2193,
//2190,2188,2186,2183,2181,2178,2176,2173,2170,2168,2165,2162,2160,2157,2154,2151,2148,2145,2142,2139,
//2136,2133,2130,2127,2124,2121,2118,2115,2112,2109,2105,2102,2099,2096,2092,2089,2086,2083,2079,2076,
//2073,2069,2066,2063,2059,2056,2053,2049,2046,2042,2039,2036,2032,2029,2026,2022,2019,2016,2012,2009,
//2006,2003,1999,1996,1993,1990,1986,1983,1980,1977,1974,1971,1968,1965,1962,1959,1956,1953,1950,1947,
//1944,1941,1938,1935,1933,1930,1927,1925,1922,1919,1917,1914,1912,1909,1907,1905,1902,1900,1898,1896,
//1893,1891,1889,1887,1885,1883,1881,1880,1878,1876,1874,1873,1871,1869,1868,1867,1865,1864,1862,1861,
//1860,1859,1858,1857,1856,1855,1854,1853,1852,1852,1851,1850,1850,1849,1849,1849,1848,1848,1848,1848,
//1848,1848,1848,1848,1848,1848,1848,1848,1849,1849,1850,1850,1851,1851,1852,1853,1854,1854,1855,1856,
//1857,1858,1859,1861,1862,1863,1864,1866,1867,1869,1870,1872,1873,1875,1877,1879,1880,1882,1884,1886,
//1888,1890,1892,1894,1897,1899,1901,1903,1906,1908,1911,1913,1916,1918,1921,1923,1926,1929,1931,1934,
//1937,1940,1942,1945,1948,1951,1954,1957,1960,1963,1966,1969,1972,1975,1979,1982,1985,1988,1991,1995,
//1998,2001,2004,2008,2011,2014,2017,2021,2024,2027,2031,2034,2037,2041,2044
// };


int counum=0;

s16 adc_flag=0;
void IRQ0_Handler(void)             //PWM Interrupt
{
//   SysTick_Config(0xffffff);
		
	//    DIV_Div((0xffffff - SysTick->VAL), 48);    
	//    SysTick_Config(0xffffff);
	//    while(DIV_Div_IsBusy());
	//    DIV_Div_Result(&timecout, &rem);

	//    TIMR_Start(TIMR0);

	//	iADCisrCnt++;

	if( uGF.bit.RunMotor )
	{
		//SysTick_Config(0xffffff);

		MeasCompCurr(&ParkParm,&MeasCurrParm,ADIa,ADIb);   //2us

		// Calculate commutation angle using estimator
		CalculateParkAngle();	//26us

		// Calculate qId,qIq from qSin,qCos,qIa,qIb
		ClarkePark(&ParkParm);  //2us

		// Calculate control values
		DoControl();	   //11us->8us

		// Calculate qSin,qCos from qAngle
		SinCos(&ParkParm);  //4us

		// Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
		InvPark(&ParkParm);   //2us  

		// Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
		CalcRefVec(&ParkParm,&SVGenParm);  //2us

		// Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
		CalcSVGen(&SVGenParm); 	//19->4us 
        
        phase_current_max_check(&motor_fault);
        //Phase_current_deviation(1);
	} 
//    timecout = 0xffffff - SysTick->VAL;           	
//    DIV_Div((0xffffff - SysTick->VAL), 48);  
//    while(DIV_Div_IsBusy());
//    DIV_Div_Result(&timecout, &rem);

	PWMG->IRAWST = (0x01 << PWMG_IRAWST_NEWP0A_Pos);//PWMG->IRAWST |= ((0x01 << PWMG_IRAWST_HEND0A_Pos) | (0x01 << PWMG_IRAWST_HEND1A_Pos)) ;     
}

#define timer3_times	96000000	// 2秒
u32 inputfreqcnt = 0;
void IRQ4_Handler(void)
{
	u32 Timval;
	
#if 0
	EXTI_Clear(GPIOE, PIN2);
	
	if(++InpFreqCnt.InputFreqcnt >= INPFRENUM)//必须满足最低转速时的计算时间
	{
		Timval = TIMR3->CVAL;
		TIMR_Stop(TIMR3);
		InpFreqCnt.Timercnt = 96000000 - Timval;      //获取定时器计数值


		InpFreqCnt.CountStartFlag = 2;          //停止定时器计数	
		InpFreqCnt.InputFreqcnt = 0;
	}
	else if(InpFreqCnt.InputFreqcnt <= 1)
	{	
		TIMR3->LDVAL = 96000000; //TIMR_Init(TIMR3, TIMR_MODE_TIMER, 96000000, 0);//2S周期
		TIMR_Start(TIMR3);
	}
#endif	

#if 1 
    
	++inputfreqcnt;
	if(inputfreqcnt <= 1)
	{	
		TIMR3->LDVAL = timer3_times; //TIMR_Init(TIMR3, TIMR_MODE_TIMER, 96000000, 0);//2S周期
		TIMR_Start(TIMR3);
	}
    EXTI_Clear(GPIOE, PIN2);
#endif
}

u16 speedmax = 790;
void IRQ1_Handler(void)         //Timer Interrupt
{
	//    SysTick_Config(0xffffff);

	if( TIMRG->IF & TIMRG_IF_TIMR0_Msk )    //Timer0 Interrupt      //单次模式
	{
		//        GPIO_InvBit(GPIOE,PIN0);
		ADC->START = (0x01 << ADC_START_GO_Pos);	//Turn on A/D module	
		while( adc_flag==0 )
		adc_flag = (ADC->CH[4].STAT & ADC_STAT_EOC_Msk);
		ADC->CH[4].STAT = (1 << ADC_STAT_EOC_Pos);  //Clear status

		//        GPIO_InvBit(GPIOE,PIN0);

		ADIa = ADC->CH[AD_Ia].DATA & ADC_DATA_VALUE_Msk;    //Channel 3
		ADIb = ADC->CH[AD_Ib].DATA & ADC_DATA_VALUE_Msk;//Channel 4

		TIMR_Stop(TIMR0);
		TIMR_INTClr(TIMR0);
	}
    
	if( TIMRG->IF & TIMRG_IF_TIMR1_Msk )    //Timer1 Interrupt
	{   
        if( VDC_status_tiems++ >=10 )   //1s定时
        {
            VDC_status_tiems = 10;
            VDC_status = 1;     //母线电压稳定
        }
        if( VDC_status )
            SysLed_Twinkle(&motor_fault);
		TIMR_INTClr(TIMR1);
	}
    
	if( TIMRG->IF & TIMRG_IF_TIMR2_Msk )    //Timer2 Interrupt
	{
		++reset_delaytimes;
		if( reset_delaytimes >= 10 )
		{
			reset_delaytimes = 0;
			SPREF += 50;
			if( SPREF >=speedmax )
				SPREF = speedmax;
		}
		if( SPREF >=speedmax )
			GPIO_InvBit(GPIOB,PIN1);
		TIMR_INTClr(TIMR2);
	}
    
    if( TIMRG->IF & TIMRG_IF_TIMR3_Msk )    //Timer3 Interrupt
	{
        TIMR_Stop(TIMR3);
        EXTI_Clear(GPIOE, PIN2);
        EXTI_Close(GPIOE, PIN2);
        
		InpFreqCnt.Timercnt = timer3_times;
		InpFreqCnt.InputFreqcnt = inputfreqcnt;
		InpFreqCnt.CountStartFlag = 2;			//停止定时器计数	
		inputfreqcnt = 0;
        
        TIMR_INTClr(TIMR3);
    }

//    DIV_Div((0xffffff - SysTick->VAL), 48);  
//    while(DIV_Div_IsBusy());
//    DIV_Div_Result(&timecout, &rem);
}

u8 SP_High,SP_Low;
u8 s1,s2,s3;
void IRQ2_Handler(void)
{
	SP_High = UART_ReadByte(UART0);
	SP_Low = UART_ReadByte(UART0);
	SPREF = (SP_High&0xf)*100 + (SP_Low>>4)*10 + (SP_Low&0xf);
}


void IRQ5_Handler(void)
{	
	SpeedTotal += ADC->CH[AD_Speed].DATA & ADC_DATA_VALUE_Msk;//Channel 2
	Speedtimes++;
	if( Speedtimes >= 4096 )
	{
		SpeedV = SpeedTotal>>12;
		SpeedTotal = 0;
		Speedtimes = 0;
	}

	VdcTotal += ADC->CH[AD_Vdc].DATA & ADC_DATA_VALUE_Msk;//Channel 0
	Vdctimes++;
	if( Vdctimes >= 4095 )
	{
		TargetDCbus = VdcTotal>>12;
		VdcTotal = 0;
		Vdctimes = 0;
	}
    
    motor_fault.IPM_module->Ad_Temperature_total += ADC->CH[AD_IPM_Temperature].DATA & ADC_DATA_VALUE_Msk;//Channel 1
    motor_fault.IPM_module->Ad_Temperature_times++;
    if( motor_fault.IPM_module->Ad_Temperature_times >=1024 )
    {
        motor_fault.IPM_module->Ad_Temperature_value = motor_fault.IPM_module->Ad_Temperature_total>>10;
        motor_fault.IPM_module->Ad_Temperature_total = 0;
        motor_fault.IPM_module->Ad_Temperature_times = 0;
    }
         
    
	ADIa = ADC->CH[AD_Ia].DATA & ADC_DATA_VALUE_Msk;    //Channel 3
	ADIb = ADC->CH[AD_Ib].DATA & ADC_DATA_VALUE_Msk;//Channel 4

	ADC_IntEOCClr(ADC, ADC_CH4);	//清除中断标志
    
}

bool SetupParm(void)
{
	u32 Ad0Sum = 0,Ad1Sum = 0,Ad2Sum = 0;	
    u32 i = 0;
	static u8 AdCn = 0;
	
	PWM_InitStructure  PWM_initStruct;
	ADC_InitStructure ADC_initStruct;
    
	// Setup required parameters

	PWM_per = LOOPINTCY;
	T0_t = LOOPINTCY/2;
	PWM_CLOCK_CYCLE = LOOPINTCY/2;	
	
	// ============= Open Loop ======================
	// Motor End Speed Calculation
	// MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	// Then, * 65536 which is a right shift done in "void CalculateParkAngle(void)"
	// ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS* 65536* LOOPTIMEINSEC * 65536/ 60.0;//角度   
	MotorParm.LockTime = LOCKTIME;

	// Scaling constants: Determined by calibration or hardware design.
	ReadADCParm.qK      = DQK;    

	MeasCurrParm.qKa    = DQKA;    
	MeasCurrParm.qKb    = DQKB;   	

	// ============= SVGen ===============
	// Set PWM period to Loop Time 
	SVGenParm.iPWMPeriod = PWM_per;      

	// ============= Motor PWM ======================
	PWM_initStruct.clk_div = PWM_CLKDIV_1;		

	PWM_initStruct.mode = PWM_MODE_COMPL_CALIGN;		//A路和B路为一路互补输出，中心对齐		
	PWM_initStruct.cycleA = PWM_per/2;				
	PWM_initStruct.hdutyA = PWM_per/6;
	PWM_initStruct.deadzoneA = DDEADTIME;
	PWM_initStruct.initLevelA = 0;
	PWM_initStruct.cycleB = PWM_per/2;		
	PWM_initStruct.hdutyB = PWM_per/2 - PWM_per/6; 
	PWM_initStruct.deadzoneB = DDEADTIME;
	PWM_initStruct.initLevelB = 1;
	PWM_initStruct.HEndAIEn = 0;
	PWM_initStruct.NCycleAIEn = 1;
	PWM_initStruct.HEndBIEn = 0;
	PWM_initStruct.NCycleBIEn = 0;	
	PWM_Init(PWM0, &PWM_initStruct);

	PWM_initStruct.cycleA = PWM_per / 2;				
	PWM_initStruct.hdutyA = 1;
	PWM_initStruct.cycleB = PWM_per/2;		
	PWM_initStruct.hdutyB = PWM_per/2 - 1; 
	PWM_initStruct.NCycleAIEn = 0;
	PWM_Init(PWM1, &PWM_initStruct);
	PWM_Init(PWM2, &PWM_initStruct);

	PWMG->ADTRG0A = 0;
	PWMG->ADTRG0A |= (0<<16); // 后 半周期生效，0前半周期生效
	PWMG->ADTRG0A |= (u32)(LOOPINTCY/2-100) ;    //(0x00 & 0xffff);     
	PWMG->ADTRG0A |= (1<<17);   //使能PWM触发	

	IRQ_Connect(IRQ0_15_PWM, IRQ0_IRQ, 1);      //set PWM IRQ priority
	
	/*****FG frequence output PWM init start*****/
	
	PWM_initStruct.clk_div = PWM_CLKDIV_1;			//分频后为3M	
	PWM_initStruct.mode = PWM_MODE_INDEP;		//A路和B路为独立输出		
	PWM_initStruct.cycleA = LOOPINTCY;				
	PWM_initStruct.hdutyA = LOOPINTCY;
	PWM_initStruct.initLevelA = 1;
	PWM_initStruct.HEndAIEn = 0;
	PWM_initStruct.NCycleAIEn = 0;
	PWM_Init(PWM3, &PWM_initStruct);

	/******FG frequence output PWM init end*****/
	
	// Center aligned PWM.
	// Note: The PWM period is set to dLoopInTcy/2 but since it counts up and 
	// and then down => the interrupt flag is set to 1 at zero => actual 
	// interrupt period is dLoopInTcy

	TIMR_Init(TIMR0, TIMR_MODE_TIMER, T0_t, 1);
	TIMR_Init(TIMR1, TIMR_MODE_TIMER, 4800000, 1);  //100ms，用于LED指示
	TIMR_Init(TIMR2, TIMR_MODE_TIMER, 48000000, 1);
	IRQ_Connect(IRQ0_15_TIMR, IRQ1_IRQ, 0);      //set TIMR IRQ priority
	

	// ============= ADC - Measure Current & Pot ======================
	ADC_initStruct.clk_src = ADC_CLKSRC_HRC_DIV4;
	ADC_initStruct.channels = ADC_CH0 | ADC_CH3 | ADC_CH4;
	ADC_initStruct.trig_src = ADC_TRIGSRC_SW;
	ADC_initStruct.Continue = 1;					//连续模式
	ADC_initStruct.EOC_IEn = 0;	
	ADC_initStruct.OVF_IEn = 0;
	ADC_Init(ADC, &ADC_initStruct);					//配置ADC	
	ADC_Open(ADC);									//使能ADC

    TIMR_Start(TIMR1);      //开启定时器，指示系统状态
	while( VDC_status == 0 );   //待电压稳定后再检测电压

	ADC_Start(ADC);									//start ADC

	// Initial Current offsets
	while(1)
	{
		if(AdCn < 160)
		{				
			AdCn++;	
			if(AdCn % 40 == 0)
			{
				Ad0Sum += ADC->CH[AD_Ia].DATA & ADC_DATA_VALUE_Msk;
				Ad1Sum += ADC->CH[AD_Ib].DATA & ADC_DATA_VALUE_Msk;
				Ad2Sum += ADC->CH[AD_Vdc].DATA & ADC_DATA_VALUE_Msk; 
			}

		}
		else if(AdCn >= 160)
		{
			InitMeasCompCurr(&MeasCurrParm,Ad0Sum >> 2, Ad1Sum >> 2);
			// Target DC Bus, without sign.
			TargetDCbus = Ad2Sum >> 2;

			break;
		}

	}	

	ADC_initStruct.clk_src = ADC_CLKSRC_HRC_DIV4;
	ADC_initStruct.channels = ADC_CH0| ADC_CH1| ADC_CH2| ADC_CH3 | ADC_CH4;
	ADC_initStruct.trig_src = ADC_TRIGSRC_PWM;//ADC_TRIGSRC_SW;
	ADC_initStruct.Continue = 0;					//单次模式
	ADC_initStruct.EOC_IEn = ADC_CH4;	
	ADC_initStruct.OVF_IEn = 0;
	ADC_Init(ADC, &ADC_initStruct);					//配置ADC	
	ADC_Open(ADC);									//使能ADC
	IRQ_Connect(IRQ0_15_ADC, IRQ5_IRQ, 0);
    
	          
	return FALSE;
}


void CountInputFreq_Init(void)
{

	GPIO_Init(GPIOE, PIN2, 0, 0, 0, 0);			//输入    
	EXTI_Init(GPIOE, PIN2, EXTI_FALL_EDGE);		//下降沿触发中断	
	IRQ_Connect(IRQ0_15_GPIOE2, IRQ4_IRQ, 0);	
	EXTI_Open(GPIOE, PIN2);
	TIMR_Init(TIMR3, TIMR_MODE_TIMER, timer3_times, 1);// 2s定时,开中断
}




void CalculateParkAngle (void)
{
	smc1.Ialpha = RIGHSHIFT15(ParkParm.qIalpha*175);//ParkParm.qIalpha;//////RIGHSHIFT15(ParkParm.qIalpha*9929);	//Q15
	smc1.Ibeta = RIGHSHIFT15(ParkParm.qIbeta*175);//ParkParm.qIbeta;////RIGHSHIFT15(ParkParm.qIbeta*9929);
	smc1.Valpha = RIGHSHIFT15(ParkParm.qValpha*32767);//ParkParm.qValpha;//
	smc1.Vbeta = RIGHSHIFT15(ParkParm.qVbeta*32767);//ParkParm.qVbeta;//

	Loopflag = uGF.bit.OpenLoop;

	SMC_Position_Estimation(&smc1);

	if(uGF.bit.OpenLoop)	
	{
		if (Startup_Lock < MotorParm.LockTime)
		{
			Startup_Lock += 1;	// This variable is incremented until
			stas = 0;
		}
		// lock time expires, them the open loop
		// ramp begins
		else if (Startup_Ramp < MotorParm.EndSpeed)
		{
			// Ramp starts, and increases linearly until EndSpeed is reached.
			// After ramp, estimated theta is used to commutate motor.
			Startup_Ramp += DELTA_STARTUP_RAMP;
			stas = 1;
		}
		else
		{
			// This section enables closed loop, right after open loop ramp.
			//?????????,?????????,??????			
			uGF.bit.ChangeMode = 1;
			uGF.bit.OpenLoop = 0;

			// Difference between force angle and estimated theta is saved,
			// so a soft transition is made when entering closed loop.
			Theta_error = ParkParm.qAngle - smc1.Theta;            
			stas = 2;
		}		
		ParkParm.qAngle += (Startup_Ramp>>16);
	}
	else
	{
		// This value is used to transition from open loop to closed looop. 
		// At the end of open loop ramp, there is a difference between 
		// forced angle and estimated angle. This difference is stored in 
		// Theta_error, and added to estimated theta (smc1.Theta) so the 
		// effective angle used for commutating the motor is the same at 
		// the end of open loop, and at the begining of closed loop. 
		// This Theta_error is then substracted from estimated theta 
		// gradually in increments of 0.05 degrees until the error is less
		// than 0.05 degrees.

		if( Run_nomal == 1 )
		{
			ParkParm.qAngle = smc1.Theta + Theta_error;     
		}

		if (Theta_error > _0_05DEG ||Theta_error < (0- _0_05DEG))
		{
			if (Theta_error < 0)
			Theta_error += _0_05DEG;
			else
			Theta_error -= _0_05DEG;
		}

	}
	return;
}

void SetupControlParameters(void)
{
	// ============= PI D Term ===============      
	PIParmD.qKp = DKP;       
	PIParmD.qKi = DKI;              
	PIParmD.qKc = DKC;       
	PIParmD.qOutMax = DOUTMAX;
	PIParmD.qOutMin = 0 - DOUTMAX;//-3000;//

	InitPI(&PIParmD);

	// ============= PI Q Term ===============
	PIParmQ.qKp = QKP;    
	PIParmQ.qKi = QKI;
	PIParmQ.qKc = QKC;
	PIParmQ.qOutMax = QOUTMAX;
	PIParmQ.qOutMin = PIParmQ.qOutMax/100;//0 -PIParmQ.qOutMax;

	InitPI(&PIParmQ);

	// ============= PI W Term ===============
	PIParmW.qKp = WKP;       
	PIParmW.qKi = WKI;       
	PIParmW.qKc = WKC;       
	PIParmW.qOutMax = WOUTMAX;   
	PIParmW.qOutMin = 400;//

	InitPI(&PIParmW);

	return;
}

void DebounceDelay(void)
{
	long i;
	for (i = 0;i < 100000;i++)
	;
	return;
}

// NOTE:
//
// If Input power supply has switching frequency noise, for example if a
// switch mode power supply is used, Voltage Ripple Compensation is not
// recommended, since it will generate spikes on Vd and Vq, which can
// potentially make the controllers unstable.

s32 VoltRippleComp(s32 Vdq)
{
	s32 CompVdq,DivTemp;
	// DCbus is already updated with new DC Bus measurement
	// in ReadSignedADC0 subroutine.
	//
	// If target DC Bus is greater than what we measured last sample, adjust
	// output as follows:
	//
	//                  TargetDCbus - DCbus
	// CompVdq = Vdq + --------------------- * Vdq
	//                         DCbus
	//
	// If Measured DCbus is greater than target, then the following compensation
	// is implemented:
	//
	//            TargetDCbus 
	// CompVdq = ------------- * Vdq
	//               DCbus
	//
	// If target and measured are equal, no operation is made.
	//
	DivTemp = (TargetDCbus - DCbus)/DCbus;
	if (TargetDCbus > DCbus)
	//	CompVdq = Vdq + FracMpy(FracDiv(TargetDCbus - DCbus, DCbus), Vdq);
	CompVdq = Vdq + DivTemp * Vdq;
	else if (DCbus > TargetDCbus)
	// 	CompVdq = FracMpy(FracDiv(TargetDCbus, DCbus), Vdq);
	CompVdq = DivTemp * Vdq;
	else
	CompVdq = Vdq;

	DCbus = TargetDCbus;

	return CompVdq;
}

void CountInputFreq(void)       //计算输入频率
{
	s32 Di, Vi, Qi, Ri;

	Di = InpFreqCnt.InputFreqcnt * (48000000>>10);
	Vi = InpFreqCnt.Timercnt>>10;      
	Qi = Di/Vi;
	InpFreqCnt.InputFreq = Qi;
}

void calculate_rotate_speed(InpFreq *pfreq)
{
	if( InpFreqCnt.CountStartFlag == 2 )
	{
		InpFreqCnt.CountStartFlag = 0;
		CountInputFreq();

		SPREF = pfreq->InputFreq * 60;

		if(smc1.OmegaFltred > 0)
		{
			fg_outfrequence = (smc1.OmegaFltred*SPEEDLOOPFREQ/POLEPAIRS)>>16;
			
			PWM3->PERA = 48000000/fg_outfrequence;
			PWM3->HIGHA = PWM3->PERA >> 1;
		}
        
       EXTI_Open(GPIOE, PIN2);  //再次打开外部中断 
	}
}

