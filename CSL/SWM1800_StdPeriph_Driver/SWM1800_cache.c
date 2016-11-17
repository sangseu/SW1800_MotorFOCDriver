/****************************************************************************************************************************************** 
* 文件名称: SWM1800_cache.c
* 功能说明:	SWM1800单片机CACHE操作驱动函数
* 技术支持:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* 注意事项:
* 版本日期: V1.0.0		2016年1月30日
* 升级记录: 
*
*
*******************************************************************************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION 
* REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE 
* FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT 
* OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
* -ECTION WITH THEIR PRODUCTS.
*
* COPYRIGHT 2012 Synwit Technology 
*******************************************************************************************************************************************/
#include "SWM1800.h"
#include "SWM1800_cache.h"

/****************************************************************************************************************************************** 
* 函数名称:	CACHE_Init()
* 功能说明:	CHCHE初始化
* 输    入: CACHE_InitStructure * initStruct	包含CACHE相关设定值的结构体
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void CACHE_Init(CACHE_InitStructure * initStruct)
{
	CACHE->CR = (1 << CACHE_CR_RST_Pos) |
				(initStruct->Alg << CACHE_CR_ALG_Pos);
	
	CACHE->LOCKTHR = initStruct->Threshold;
	
	CACHE->IF = 0x07;	//清除中断标志
	CACHE->IE = (1                    << CACHE_IE_EN_Pos)      |
				(initStruct->ResetIEn << CACHE_IE_RESET_Pos)   |
				(0                    << CACHE_IE_INVALID_Pos) |
				(0                    << CACHE_IE_PREFETCH_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	CACHE_Reset()
* 功能说明:	CHCHE复位，清除CHCHE所有标志和内容
* 输    入: 无
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void CACHE_Reset(void)
{
	CACHE->CR |= (1 << CACHE_CR_RST_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	CACHE_Prefetch()
* 功能说明:	CHCHE预取锁存
* 输    入: uint32_t addr		将Flash中addr开始的128字节代码预取到Cache中并锁定
* 输    出: uint32_t			0 预取锁存成功    1 预取失败    2 锁存失败    3 未知错误
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t CACHE_Prefetch(uint32_t addr)
{
	CACHE->PREFCR = (1 << CACHE_PREFCR_EN_Pos) |
					(addr << CACHE_PREFCR_ADDR_Pos);
	
	while((CACHE->PREFSR & CACHE_PREFSR_FINISH_Msk) == 0);
	
	if((CACHE->PREFSR & CACHE_PREFSR_LOCKSUCC_Msk) != 0)  return 0;
	else if((CACHE->PREFSR & CACHE_PREFSR_FAIL_Msk) != 0) return 1;
	else if((CACHE->PREFSR & CACHE_PREFSR_SUCC_Msk) != 0) return 2;
	else                                                  return 3;
}

/****************************************************************************************************************************************** 
* 函数名称:	CACHE_PrefetchSlotNumber()
* 功能说明:	获取预取锁存操作锁定的Slot的编号
* 输    入: 无
* 输    出: uint32_t			锁定的Slot号
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t CACHE_PrefetchSlotNumber(void)
{
	return (CACHE->PREFSR & CACHE_PREFSR_SLOTNUM_Msk) >> CACHE_PREFSR_SLOTNUM_Pos;
}

/****************************************************************************************************************************************** 
* 函数名称:	CACHE_Invalid()
* 功能说明:	将锁存在Cache中的代码失效，从而释放Cache Slot
* 输    入: uint32_t addr		将Flash中addr开始的128字节代码在Cache中的拷贝失效，从而释放Cache Slot
* 输    出: uint32_t			0 失效成功，将Slot标记为无效    1 失效失败，指定代码不在Cache中    2 未知错误
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t CACHE_Invalid(uint32_t addr)
{
	CACHE->INVALID = (1 << CACHE_INVALID_EN_Pos) |
					 (addr << CACHE_INVALID_ADDR_Pos);
	
	while((CACHE->INVALIDSR & CACHE_INVALIDSR_FINISH_Msk) == 0);
	
	if((CACHE->INVALIDSR & CACHE_INVALIDSR_SUCC_Msk) != 0)      return 0;
	else if((CACHE->INVALIDSR & CACHE_INVALIDSR_FAIL_Msk) != 0) return 1;
	else                                                        return 2;
}

/****************************************************************************************************************************************** 
* 函数名称:	CACHE_InvalidSlotNumber()
* 功能说明:	获取失效操作释放的Slot的编号
* 输    入: 无
* 输    出: uint32_t			失效的Slot号
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t CACHE_InvalidSlotNumber(void)
{
	return (CACHE->INVALIDSR & CACHE_INVALIDSR_SLOTNUM_Msk) >> CACHE_INVALIDSR_SLOTNUM_Pos;
}
