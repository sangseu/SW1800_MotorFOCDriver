#include "Park.h"
#include "General.h"

#include "SWM1800.h"

/*
*表格的数值根据sin(360*n/128)扩大32767倍算出
*n=(0..128)
*/
const signed int SinTable[]=
{
	0,
	1608,
	3212,
	4808,
	6393,
	7962,
	9512,
	11039,
	12539,
	14010,
	15446,
	16846,
	18204,
	19519,
	20787,
	22005,
	23170,
	24279,
	25329,
	26319,
	27245,
	28105,
	28898,
	29621,
	30273,
	30852,
	31356,
	31785,
	32137,
	32412,
	32609,
	32728,
	32767,
	32728,
	32609,
	32412,
	32137,
	31785,
	31356,
	30852,
	30273,
	29621,
	28898,
	28105,
	27245,
	26319,
	25329,
	24279,
	23170,
	22005,
	20787,
	19519,
	18204,
	16846,
	15446,
	14010,
	12539,
	11039,
	9512,
	7962,
	6393,
	4808,
	3212,
	1608,
	0,
	-1608,
	-3212,
	-4808,
	-6393,
	-7962,
	-9512,
	-11039,
	-12539,
	-14010,
	-15446,
	-16846,
	-18204,
	-19519,
	-20787,
	-22005,
	-23170,
	-24279,
	-25329,
	-26319,
	-27245,
	-28105,
	-28898,
	-29621,
	-30273,
	-30852,
	-31356,
	-31785,
	-32137,
	-32412,
	-32609,
	-32728,
	-32767,
	-32728,
	-32609,
	-32412,
	-32137,
	-31785,
	-31356,
	-30852,
	-30273,
	-29621,
	-28898,
	-28105,
	-27245,
	-26319,
	-25329,
	-24279,
	-23170,
	-22005,
	-20787,
	-19519,
	-18204,
	-16846,
	-15446,
	-14010,
	-12539,
	-11039,
	-9512,
	-7962,
	-6393,
	-4808,
	-3212,
	-1608
};
#define table_size 128

void SinCos(tParkParm *pParkParm)
{
	s32 angle,angle_delta;
	u32 angletemp = 0;
	u16 anglel;
	u16 index1,index2;

	angle = pParkParm->qAngle;
    if(angle < 0)
    {
        angle +=65536;
//		angle = ~(0-angle) + 1;
    }

	angletemp = ((u32)angle) << 7;				//angle*table_size
	anglel = angletemp & 0xffff;			//
	index1 = (angletemp & 0xffff0000)>>16;		//hight word is index
	if(anglel == 0)					//if anglel is zero,get sin and cos value in the table directly
	{
		pParkParm->qSin = SinTable[index1];
        index2 = (index1 + 32)&0x7f;
		pParkParm->qCos = SinTable[index2];
	}
	else
	{
		index2 = (index1 + 1)&0x7f;
        angle_delta = 	SinTable[index2] - SinTable[index1];
		pParkParm->qSin = SinTable[index1] + ((anglel * angle_delta) >> 16);
		index1 = (index1 + 32)&0x7f;
        index2 = (index1 + 1)&0x7f;
		angle_delta = SinTable[index2] - SinTable[index1];
		pParkParm->qCos = SinTable[index1] + ((anglel * angle_delta) >> 16);         
	}
	
}

#if 0
#define TableSize 46
/*
*180度对应32767
*/
/****************************************
SinTable表格中的数据0，2，4，...90度
对应的正弦值扩大32767(Q15格式)

****************************************/

// 正弦值扩大32768
const unsigned int SinTable[]=
{
	0,1144,2286,3425,4560,5690,6813,7927,9032,10126,
	11207,12275,13328,14365,15384,16384,17364,18324,19261,20174,
	21063,21926,22763,23571,24351,25102,25822,26510,27166,27789,
	28378,28932,29452,29935,30382,30792,31164,31499,31795,32052,
	32270,32449,32588,32688,32748,32768
};//根据线性插值查找法计算
//角度相对应的SIN与COS值
int AngleT; 
u32 MiddleN;
u32 Quad=1;
u32 sinnum=0;
u32 cosnum=0;
u32 N1,N2;
void SinCos(tParkParm *pParkParm)
{
	int valtemp;

	sinnum=0;
	cosnum=0;
	valtemp = Q15(2/180);
	

	
	AngleT= pParkParm->qAngle;	//
	
	if(AngleT > 16384 && AngleT <= 32768)
	{
			AngleT = 32768 - AngleT;
			Quad=2;
	}
	else if(AngleT > 32768 && AngleT <= 49152)
	{
			AngleT = AngleT - 32768;
			Quad=3;
	}
	else if(AngleT > 49152 && AngleT <= 65536)
	{
			AngleT = 65536 - AngleT ;
			Quad=4;
	}
	else
	{
			Quad=1;
	}
	
	MiddleN = 45;
	if( Q15(MiddleN) >= AngleT*180 )
	{
		if( Q15(MiddleN) >= AngleT*360 )
			sinnum = 0;
		else
			sinnum = MiddleN/4;
	}
	else
	{
		if( Q15(MiddleN) >= AngleT*120 )
			sinnum = MiddleN/2;
		else
			sinnum = MiddleN*3/4;
	}
	
// 	while( !( (Q15(sinnum*2/180)<=AngleT) && (AngleT<=Q15((sinnum+1)*2/180)) ) )
	while( ( (Q15(sinnum)<=AngleT*90) && (AngleT*90<=Q15((sinnum+1))) )==0 )
	{
		sinnum++;
	}
	cosnum = TableSize - sinnum -2;

	
	if( Quad == 1 )
	{
			pParkParm->qSin = SinTable[sinnum] + (SinTable[sinnum+1]-SinTable[sinnum])*(AngleT-Q15(sinnum/90))/valtemp;
 			pParkParm->qCos = SinTable[cosnum] + (SinTable[cosnum+1]-SinTable[cosnum])*(16384-AngleT-Q15(cosnum/90))/valtemp;
	}
	else if( Quad == 2 )
	{
			pParkParm->qSin = SinTable[sinnum] + (SinTable[sinnum+1]-SinTable[sinnum])*(AngleT-Q15(sinnum/90))/valtemp;
			pParkParm->qCos = 0-( SinTable[cosnum] + (SinTable[cosnum+1]-SinTable[cosnum])*(16384-AngleT-Q15(cosnum/90))/valtemp );
	}
	else if( Quad == 3 )
	{
			pParkParm->qSin = 0-( SinTable[sinnum] + (SinTable[sinnum+1]-SinTable[sinnum])*(AngleT-Q15(sinnum/90))/valtemp );
			pParkParm->qCos = 0-( SinTable[cosnum] + (SinTable[cosnum+1]-SinTable[cosnum])*(16384-AngleT-Q15(cosnum/90))/valtemp );
	}
	else if( Quad == 4 )
	{
			pParkParm->qSin = 0-( SinTable[sinnum] + (SinTable[sinnum+1]-SinTable[sinnum])*(AngleT-Q15(sinnum/90))/valtemp );
			pParkParm->qCos = SinTable[cosnum] + (SinTable[cosnum+1]-SinTable[cosnum])*(16384-AngleT-Q15(cosnum/90))/valtemp;
	}	
}
#endif

#if 0
unsigned int sqrt_16(unsigned long M) 
{ 
     unsigned int N, i; 
     unsigned long tmp, ttp;    // ??????? 
     if (M == 0)                // ????,??????0 
         return 0; 

     N = 0; 

     tmp = (M >> 30);           // ?????:B[m-1] 
     M <<= 2; 
     if (tmp > 1)               // ????1 
     { 
         N ++;                  // ??????1,??????0 
         tmp -= N; 
     } 

     for (i=15; i>0; i--)       // ????15? 
     { 
         N <<= 1;               // ???? 

         tmp <<= 2; 
         tmp += (M >> 30);      // ?? 

         ttp = N; 
         ttp = (ttp<<1)+1; 

         M <<= 2; 
         if (tmp >= ttp)        // ???? 
         { 
             tmp -= ttp; 
             N ++; 
         } 

     } 

     return N; 
}
#endif

u32 sqrt_16(u32 radicand) 
{ 
    u32 root_val;
    
    DIV_Root(radicand, 0);
    while(DIV_Root_IsBusy());
    root_val = DIV_Root_Result();
    
    return root_val; 
}

void DIV_Fun(int Divd, int Divs, int *Quo, int *Rem)
{
    u32 divdend,divsor,Quot,Remain;
    u32 Quo_sign_flag;
    
    if( Divd>=0 && Divs>0 )
    {
        divdend = Divd;
        divsor = Divs;
        Quo_sign_flag = 0;
    }
    else if( Divd>=0 && Divs<0 )
    {
        divdend = Divd;
        divsor = -Divs;
        Quo_sign_flag = 1;
    }
    else if( Divd<=0 && Divs>0 )
    {
        divdend = -Divd;
        divsor = Divs;
        Quo_sign_flag = 1;
    }
    else if( Divd<=0 && Divs<0 )
    {
        divdend = -Divd;
        divsor = -Divs;
        Quo_sign_flag = 0;
    }
    else
    {
        divdend = 0;
        divsor = 1;
        Quo_sign_flag = 0;
    }
    
    DIV_Div(divdend, divsor);    
    while(DIV_Div_IsBusy());
    
    *Rem = DIV->REMAIN;
    if( Quo_sign_flag==1 )
        *Quo = -DIV->QUO; 
    else
        *Quo = DIV->QUO;       
}

