#include "svgen.h"
#include "General.h"
#include "SWM1800.h"
#include "UserParms.h"


#define Duty_KP 200		//----
#define Duty_KI 20		//----
#define Duty_KD 0  		//----


extern int Duty0,Duty1,Duty2;
extern s16 Sector;
extern s16 PWM_CLOCK_CYCLE;						//----pwm_cycle，中心对称模式设置为所需周期的一半
extern s16 Elec_Angle;
extern s16 LoopFlag;

//0-60°，分1024份，精度为60°/1024=0.05859375°。
const int sin_dat[1024]={
0,1,2,3,4,5,6,7,8,9,10,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28
,29,30,31,32,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53
,54,55,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78
,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102
,103,105,106,107,108,109,110,111,112,113,114,115,115,117,118,119,120,121
,122,123,124,125,126,127,128,130,131,132,133,134,135,136,137,138,139,140
,141,142,142,144,145,146,147,148,149,150,151,152,153,154,155,156,157,159
,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177
,178,179,180,181,182,183,184,185,186,187,188,189,191,192,193,194,195,196
,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214
,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,232,233
,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251
,252,253,254,255,256,257,257,258,259,260,261,263,264,265,266,267,268,269
,270,271,272,273,274,275,276,277,278,279,280,281,282,283,284,285,286,287
,288,289,290,291,292,293,294,295,296,297,298,299,300,301,302,303,304,305
,306,307,308,309,310,311,312,313,314,315,316,317,318,319,320,321,322,323
,324,325,326,327,328,329,330,331,332,333,334,335,336,337,338,339,340,341
,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359
,360,361,362,363,364,365,366,367,368,369,370,370,371,372,373,374,375,376
,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394
,395,396,397,398,399,400,401,402,402,403,404,405,406,407,408,409,410,411
,412,413,414,415,416,417,418,419,420,421,422,423,424,425,425,426,427,428
,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,443,444,445
,446,447,448,449,450,451,452,453,454,455,456,457,458,459,459,460,461,462
,463,464,465,466,466,467,468,469,470,471,472,473,474,475,476,477,478,479
,480,481,482,483,484,485,485,486,487,488,489,490,491,492,493,494,495,496
,497,497,498,499,500,501,502,503,504,505,506,507,507,508,509,510,511,512
,513,514,515,516,517,517,518,519,520,521,522,523,524,525,526,526,527,528
,529,530,531,532,533,534,535,535,536,537,538,539,540,541,542,543,543,544
,545,546,547,548,549,550,550,551,552,553,554,555,556,557,558,558,559,560
,561,562,563,564,565,565,566,567,568,569,570,571,572,572,573,574,575,576
,577,578,578,579,580,581,582,583,584,584,585,586,587,588,589,590,590,591
,592,593,594,595,596,596,597,598,599,600,601,602,602,603,604,605,606,607
,607,608,609,610,611,612,613,613,614,615,616,617,618,618,619,620,621,622
,623,623,624,625,626,627,628,628,629,630,631,632,632,633,634,635,636,637
,637,638,639,640,641,641,642,643,644,645,646,646,647,648,649,650,650,651
,652,653,654,654,655,656,657,657,658,659,660,661,662,662,663,664,665,666
,666,667,668,669,670,670,671,672,673,674,674,675,676,677,678,678,679,680
,681,681,682,683,684,685,685,686,687,688,688,689,690,691,692,692,693,694
,695,695,696,697,698,698,699,700,701,702,702,703,704,705,705,706,707,708
,708,709,710,711,711,712,713,714,714,715,716,717,717,718,719,720,720,721
,722,723,723,724,725,726,726,727,728,729,729,730,731,731,732,733,734,734
,735,736,737,737,738,739,739,740,741,742,742,743,744,745,745,746,747,747
,748,749,750,750,751,752,752,753,754,755,755,756,757,757,758,759,759,760
,761,762,762,763,764,764,765,766,766,767,768,769,769,770,771,771,772,773
,773,774,775,775,776,777,777,778,779,779,780,781,782,782,783,784,784,785
,786,786,787,788,788,789,790,790,791,792,792,793,794,794,795,796,796,797
,798,798,799,799,800,801,801,802,803,803,804,805,805,806,807,807,808,809
,809,810,810,811,812,812,813,814,814,815,816,816,817,817,818,819,819,820
,821,821,822,822,823,824,824,825,826,826,827,827,828,829,829,830,831,831
,832,832,833,834,834,835,835,836,837,837,838,838,839,840,840,841,841,842
,843,843,844,844,845,846,846,847,847,848,849,849,850,850,851,851,852,853
,853,854,854,855,855,856,857,857,858,858,859,859,860,861,861,862,862,863
,863,864,865,865,866,866,867,867,868,868,869,870,870,871,871,872,872,873
,873,874,875,875,876,876,877,877,878,878,879,879,880,880,881,882,882,883
,883,884,884,885,885,886,886};


//-------------------------DUTY PID-----------------------------------------------------------------------------------
int dutypid (int err,int *er0,int *er1) {
	int du;
	du = ((Duty_KP*(err-*er0))>>10)+((Duty_KI*err)>>10)+((Duty_KD*(err-2**er0+*er1))>>10);
	*er1 = *er0;
	*er0 = err;
	return du;
}



/******************************************************************************************************************
函数：Svpwm_Qua
功能：算出扇区号
入参：
出参：	
******************************************************************************************************************/
s16 Svpwm_Qua(s16 *angle)
{
	s16 qua=0;
	if(*angle>=lock0&&*angle<lock60)
    {
		qua=1;        
    }
	else if(*angle>=lock60&&*angle<lock120)
	{
		qua=2;
		*angle-=lock60;
	}
	else if(*angle>=lock120&&*angle<lock180)
	{
		qua=3;
		*angle-=lock120;
	}
	else if(*angle>=(-32768)&&*angle<lock240)
	{
		qua=4;
		*angle+=lock180;
	}
	else if(*angle>=lock240&&*angle<lock300)
	{
		qua=5;
		*angle+=lock120;
	}
	else if(*angle>=lock300&&*angle<=lock360)
	{
		qua=6;
		*angle+=lock60;
	}
	else
	{
		qua=1;
		*angle=0;
	}
    
	return qua;
}


void math_cal_0 (s16 a,s16 *sva,s16 *sv60_a) 
{
	s16 tem=0;
	tem = (a*3069)>>15;//tem=a*1023/lock60;//
	if(tem>1023)tem=1023;
	else if(tem<0)tem=0;
	*sva=sin_dat[tem];
	*sv60_a=sin_dat[1023-tem];
}


/*************************************************SVPWM*********************************************************/
s16 sin_a_val=0,sin60_a_val=0,cos_val=0,qua=0;
s16 T_1,T_2;
extern s16 PWM_CLOCK_CYCLE;

void Svpwm_Duty(s16 angle,s16 uout,int *t0,int *t1,int *t2)
{
	s32 Di, Vi, Qi, Ri;
    
    if(uout>(PWM_UOUT))uout=(PWM_UOUT);
	else if(uout<0)uout=0;
	Sector=Svpwm_Qua(&angle);

	math_cal_0(angle,&sin_a_val,&sin60_a_val);

	*t1=2*uout*sin60_a_val;										//UPOWER最大：24*16384，19位，cos_val最大：4095，12位
	*t2=2*uout*sin_a_val;	

    Di = *t1;
    Vi = Sq3Ux;
    DIV_Fun(Di, Vi, &Qi, &Ri);
	*t1 = Qi;    //*t1=((*t1)/Sq3Ux);  								//t1=((2*uout*cos_val)/(sqrt(3)*Ux))*Tpwm
    
    Di = *t2;
    Vi = Sq3Ux;
    DIV_Fun(Di, Vi, &Qi, &Ri);
	*t2 = Qi;    //*t2=((*t2)/Sq3Ux);  							    //t2=((2*uout*sin_val)/(sqrt(3)*Ux))*Tpwm

	*t1=((PWM_CLOCK_CYCLE)*(*t1))>>10;
	*t2=((PWM_CLOCK_CYCLE)*(*t2))>>10;

	*t0=PWM_CLOCK_CYCLE-(*t1)-(*t2);
}

int pwm_duty_0=0,pwm_duty_1=0,pwm_duty_2=0;	
/******************************************************************************************************************
函数：Svpwm_Cal
功能：将时间t0,t1,t2三个时间合理分配到PWM中
入参：qua:扇区号
		t0,t1,t2:PWM周期分成的三部分
出参：	t0,t1,t2:PWM0，PWM2，PWM4各个的作用时间
******************************************************************************************************************/
#define seven_duan
#ifdef seven_duan

void Svpwm_Cal(s16 qua,int *t0,int *t1,int *t2) {

// 	if( qua!=0 ) {
       
		switch(qua) {
			case 1:	
					pwm_duty_0 = *t1+*t2+((*t0)>>1);
					pwm_duty_1 = pwm_duty_0 - *t1;
					pwm_duty_2 = pwm_duty_1 - *t2;
					break;
			case 2:	
					pwm_duty_1 = *t1+*t2+((*t0)>>1);
					pwm_duty_0 = pwm_duty_1 - *t2;
					pwm_duty_2 = pwm_duty_0 - *t1;
					break;
			case 3:	
					pwm_duty_1 = *t1+*t2+((*t0)>>1);
					pwm_duty_2 = pwm_duty_1 - *t1;
					pwm_duty_0 = pwm_duty_2 - *t2;
					break;
			case 4:	
					pwm_duty_2 = *t1+*t2+((*t0)>>1);
					pwm_duty_1 = pwm_duty_2 - *t2;
					pwm_duty_0 = pwm_duty_1 - *t1;	
					break;
			case 5:	
					pwm_duty_2 = *t1+*t2+((*t0)>>1);
					pwm_duty_0 = pwm_duty_2 - *t1;
					pwm_duty_1 = pwm_duty_0 - *t2;
					break;
			case 6:	
					pwm_duty_0 = *t1+*t2+((*t0)>>1);
					pwm_duty_2 = pwm_duty_0 - *t2;
					pwm_duty_1 = pwm_duty_2 - *t1;	
					break;
			default:break;
		}
// 	}					 
	*t0 = pwm_duty_0;
	*t1 = pwm_duty_1;
	*t2 = pwm_duty_2;	 
}
#else
void Svpwm_Cal (int qua,int *t0,int *t1,int *t2) {
	int pwm_duty_0=0,pwm_duty_1=0,pwm_duty_2=0;	
	if(!(qua==0)) {
		switch(qua) {
			case 1:	
					pwm_duty_0 = *t1+*t2;
					pwm_duty_1 = *t2;
					pwm_duty_2 = 0;
					break;
			case 2:	
					pwm_duty_1 = *t1+*t2;
					pwm_duty_0 = *t1;
					pwm_duty_2 = 0;
					break;
			case 3:	
					pwm_duty_1 = *t1+*t2;
					pwm_duty_2 = *t2;
					pwm_duty_0 = 0;
					break;
			case 4:	
					pwm_duty_2 = *t1+*t2;
					pwm_duty_1 = *t1;
					pwm_duty_0 = 0;	
					break;
			case 5:	
					pwm_duty_2 = *t1+*t2;
					pwm_duty_0 = *t2;
					pwm_duty_1 = 0;
					break;
			case 6:	
					pwm_duty_0 = *t1+*t2;
					pwm_duty_2 = *t1;
					pwm_duty_1 = 0;	
					break;
			default:break;
		}
	}					 
	*t0 = pwm_duty_0;
	*t1 = pwm_duty_1;
	*t2 = pwm_duty_2;	 
}
#endif


s16 Ta = 0;
s16 Tb = 0;
s16 Tc = 0;
s16 Pwm0A,Pwm1A,Pwm2A;
u32 Va2andVb2;
u32 Va2andVb2temp;

extern int uout;
extern s16 Loopflag;
int uout_ref=0;
extern s16 stas;
extern s16 Run_nomal;
extern s16 Theta_error;
extern s16 sector;
 /***********************************************************************
 *                                                                     *
 *    Filename:       SVGEN.s                                          *
 *    Date:           10/01/08                                         *
 **********************************************************************/
void CalcSVGen( tSVGenParm *pSVGenParm )
{ 
 	s16 T1,T2;

    if( 1 )
	{

        if( pSVGenParm->qVr1 >= 0 )
        {       
            // (xx1)
            if( pSVGenParm->qVr2 >= 0 )
            {
                // (x11)
                // Must be Sector 3 since Sector 7 not allowed
                // Sector 3: (0,1,1)  0-60 degrees
                sector = 3000;
                T1 = pSVGenParm->qVr1;
                T2 = pSVGenParm->qVr2;
                CalcTimes(pSVGenParm,T1,T2);
//                PWM0->HIGHA = Pwm0A = Tb;
//                PWM1->HIGHA = Pwm1A = Ta;
//                PWM2->HIGHA = Pwm2A = Tc;
                PWM1->HIGHA = Pwm0A = Ta;
                PWM2->HIGHA = Pwm1A = Tb;
                PWM0->HIGHA = Pwm2A = Tc;
            }
            else
            {            
                // (x01)
                if( pSVGenParm->qVr3 >= 0 )
                {
                    // Sector 5: (1,0,1)  120-180 degrees
                    sector = 5000;
                    T1 = pSVGenParm->qVr3;
                    T2 = pSVGenParm->qVr1;
                    CalcTimes(pSVGenParm,T1,T2);
//                    PWM0->HIGHA = Pwm0A = Tc;
//                    PWM1->HIGHA = Pwm1A = Tb;
//                    PWM2->HIGHA = Pwm2A = Ta;
                    PWM1->HIGHA = Pwm0A = Tc;
                    PWM2->HIGHA = Pwm1A = Ta;
                    PWM0->HIGHA = Pwm2A = Tb;
                }
                else
                {
                    // Sector 1: (0,0,1)  60-120 degrees
                    sector = 1000;
                    T1 = (0 - pSVGenParm->qVr3);
                    T2 = (0 - pSVGenParm->qVr2);
                    CalcTimes(pSVGenParm,T1,T2);
//                    PWM0->HIGHA = Pwm0A = Ta;
//                    PWM1->HIGHA = Pwm1A = Tb;
//                    PWM2->HIGHA = Pwm2A = Tc;
                    PWM1->HIGHA = Pwm0A = Tb;
                    PWM2->HIGHA = Pwm1A = Ta;
                    PWM0->HIGHA = Pwm2A = Tc;
                }
            }
        }
        else
        {
            // (xx0)
            if( pSVGenParm->qVr2 >= 0 )
            {
                // (x10)
                if( pSVGenParm->qVr3 >= 0 )
                {
                    // Sector 6: (1,1,0)  240-300 degrees
                    sector = 6000;
                    T1 = pSVGenParm->qVr2;
                    T2 = pSVGenParm->qVr3;
                    CalcTimes(pSVGenParm,T1,T2);
//                    PWM0->HIGHA = Pwm0A = Ta;
//                    PWM1->HIGHA = Pwm1A = Tc;
//                    PWM2->HIGHA = Pwm2A = Tb;
                    PWM1->HIGHA = Pwm0A = Tb;
                    PWM2->HIGHA = Pwm1A = Tc;
                    PWM0->HIGHA = Pwm2A = Ta;
                }
                else
                {
                    // Sector 2: (0,1,0)  300-0 degrees
                    sector = 2000;
                    T1 = (0 - pSVGenParm->qVr1);
                    T2 = (0 - pSVGenParm->qVr3);
                    CalcTimes(pSVGenParm,T1,T2);
//                    PWM0->HIGHA = Pwm0A = Tb;
//                    PWM1->HIGHA = Pwm1A = Tc;
//                    PWM2->HIGHA = Pwm2A = Ta;
                    PWM1->HIGHA = Pwm0A = Ta;
                    PWM2->HIGHA = Pwm1A = Tc;
                    PWM0->HIGHA = Pwm2A = Tb;
                }
            }
            else
            {            
                // (x00)
                // Must be Sector 4 since Sector 0 not allowed
                // Sector 4: (1,0,0)  180-240 degrees
                sector = 4000;
                T1 = (0 - pSVGenParm->qVr2);
                T2 = (0 - pSVGenParm->qVr1);
                CalcTimes(pSVGenParm,T1,T2);
//                PWM0->HIGHA = Pwm0A = Tc;
//                PWM1->HIGHA = Pwm1A = Ta;
//                PWM2->HIGHA = Pwm2A = Tb;
                PWM1->HIGHA = Pwm0A = Tc;
                PWM2->HIGHA = Pwm1A = Tb;
                PWM0->HIGHA = Pwm2A = Ta;
            }
        }
    }
    else
    {  
        Elec_Angle = ParkParm.qAngle;
    
        Va2andVb2 = ParkParm.qValpha*ParkParm.qValpha + ParkParm.qVbeta*ParkParm.qVbeta ;

        Va2andVb2temp = sqrt_16(Va2andVb2);
    
        if( stas == 0 )
            uout =330;
        else
        {
            if( Loopflag == 1 )      //开环
            {
                uout_ref = 600 ;
                
                if( uout < uout_ref )
                    uout += 1;
                else if( uout > uout_ref )
                    uout -= 1;
            }
            else
            {
                if( Run_nomal )
                {
                    uout_ref = (PWM_UOUT*Va2andVb2temp)>>15; //uout = (PWM_UOUT*Va2andVb2temp)>>15;//  
                    
                    if( (uout-15 > uout_ref) || (uout+15 < uout_ref) )
                    {
                        if( uout < uout_ref )
                            uout += 1;
                        else if( uout > uout_ref )
                            uout -= 1; 
                                                     
// 							uout = uout_ref;
                    }
                     
//                     if( uout < UOUTMIN )
//                        uout = UOUTMIN ;
                }
//                else
//                     uout = 800;
            }
        }
     
        Svpwm_Duty(Elec_Angle,uout,&Duty0,&Duty1,&Duty2);		//根据角度，Uout算出t0,t1,t2

        //============================================
        //Step5-给各路PWM的duty寄存器赋值
        Svpwm_Cal(Sector,&Duty0,&Duty1,&Duty2);					//算出PWM0,PWM1,PWM2的作用时间	
    
        PWM0->HIGHA  =Pwm0A= Duty1;
        PWM1->HIGHA  =Pwm1A= Duty2;
        PWM2->HIGHA  =Pwm2A= Duty0;

    }
}

extern unsigned long Startup_Ramp;
/*
*since T1 is in 1.15 and PWM in integer we do multiply by
*2*PWM*T1 as integers and use upper word of results
*因为T1,T2是有符号数的Q15格式，所以
*需要乘以2转换成无符号数，然后转换
*整数
*/

void CalcTimes(tSVGenParm *pSVGenParm,s16 t1,s16 t2)
{
// 	u32 pwm_temp;
// 	pwm_temp = pSVGenParm->iPWMPeriod << 1;
// 	t1temp = (pwm_temp * t1);
// 	t2temp = (pwm_temp * t2);
// 	tt1 = (t1temp & 0xffff0000) >> 16;		//use upper word of results
// 	tt2 = (t2temp & 0xffff0000) >> 16;		//use upper word of results
// 	
// 	Tc = (pSVGenParm->iPWMPeriod - tt1 - tt2)>>1;
// 	Tb = Tc + tt1;
// 	Ta = Tb + tt2;
//     if(Ta > 2200 ) Ta = 2200;
//     if(Tb > 2200 ) Tb = 2200;
//     if(Tc > 2200 ) Tc = 2200;
   
    s32 valtemp;  
    s16 tt1,tt2;    
    tt1 = t1;
    tt2 = t2;
    if( t1+t2>= 32767 )
    {
       t1 = t1*(32767-0)/(tt1+tt2);
       t2 = t2*(32767-0)/(tt1+tt2);
    }

    valtemp = pSVGenParm->iPWMPeriod * t1;
    t1 = RIGHSHIFT15(valtemp);//t1 = ((pSVGenParm->iPWMPeriod) * t1)>>15;
    valtemp = pSVGenParm->iPWMPeriod * t2;
    t2 = RIGHSHIFT15(valtemp);//t2 = ((pSVGenParm->iPWMPeriod) * t2)>>15;


    Tc = (pSVGenParm->iPWMPeriod - t1 - t2)>>2;
    Tb = Tc + (t1>>1);
    Ta = Tb + (t2>>1);
    if(  (Ta<=0) || (Tb <=0) || (Tc<=0) ) 
    {
        if( Ta <=0 )
            Ta = 1;
        if( Tb <=0 )
            Tb = 1;
        if( Tc <=0 )
            Tc = 1;
    }
 
}   
