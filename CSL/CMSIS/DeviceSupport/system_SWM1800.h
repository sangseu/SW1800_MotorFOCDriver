#ifndef __SYSTEM_SWM1800_H__
#define __SYSTEM_SWM1800_H__

#ifdef __cplusplus
 extern "C" {
#endif

#define SYS_CLK_24MHz	0	 	//0 内部高频24MHz RC振荡器
#define SYS_CLK_6MHz	1		//1 内部高频 6MHz RC振荡器									
#define SYS_CLK_48MHz	2		//2 内部高频48MHz RC振荡器									
#define SYS_CLK_12MHz	3		//3 内部高频12MHz RC振荡器									
#define SYS_CLK_32KHz	4		//4 内部低频32KHz RC振荡器									
#define SYS_CLK_XTAL	5		//5 外部XTAL晶体振荡器（2-30MHz）

#define SYS_CLK   SYS_CLK_48MHz//SYS_CLK_24MHz


#define __HSI		(24000000UL)		//高速内部时钟
#define __LSI		(   32000UL)		//低速内部时钟
#define __HSE		(24000000UL)		//高速外部时钟


extern uint32_t SystemCoreClock;		// System Clock Frequency (Core Clock)
extern uint32_t CyclesPerUs;			// Cycles per micro second


/* Setup the microcontroller system Initialise GPIO directions and values */
extern void SystemInit(void);


/* Updates the SystemCoreClock with current core Clock retrieved from cpu registers */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif //__SYSTEM_SWM1800_H__
