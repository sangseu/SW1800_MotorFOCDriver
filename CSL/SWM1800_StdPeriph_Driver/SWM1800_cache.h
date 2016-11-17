#ifndef __SWM1800_CACHE_H__
#define __SWM1800_CACHE_H__

typedef struct {
	uint8_t  Alg;			//Cache算法，可取值CACHE_ALG_LRU、CACHE_ALG_RAND
	
	uint8_t  Threshold;		//当被LOCK的SLOT个数大于Threshold时触发Reset中断
	
	uint8_t  ResetIEn;		//当LOCKCNT > LOCKTHR时触发Reset中断，中断处理函数需要执行CACHE->CR.RST = 1复位CACHE
} CACHE_InitStructure;

#define CACHE_ALG_LRU	0
#define CACHE_ALG_RAND	1


void CACHE_Init(CACHE_InitStructure * initStruct);

void CACHE_Reset(void);

uint32_t CACHE_Prefetch(uint32_t addr);
uint32_t CACHE_PrefetchSlotNumber(void);

uint32_t CACHE_Invalid(uint32_t addr);
uint32_t CACHE_InvalidSlotNumber(void);


#endif //__SWM1800_CACHE_H__
