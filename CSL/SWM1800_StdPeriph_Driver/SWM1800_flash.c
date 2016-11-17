/****************************************************************************************************************************************** 
* 文件名称:	SWM1800_flash.c
* 功能说明:	使用芯片的IAP功能将片上Flash模拟成EEPROM来保存数据，掉电后不丢失
* 技术支持:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* 注意事项:
* 版本日期: V1.0.0		2016年1月30日
* 升级记录: 
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
#include "SWM1800_flash.h"

typedef void (*IAPFunc)(uint32_t faddr, uint32_t raddr, uint32_t cnt, uint32_t cmd);
IAPFunc IAPfunc = (IAPFunc)0x1000601;

/****************************************************************************************************************************************** 
* 函数名称:	FLASH_Erase()
* 功能说明:	FLASH扇区擦除，每个扇区4K字节
* 输    入: uint32_t addr		要擦除扇区的地址，必须4K对齐，即addr%4096 == 0
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void FLASH_Erase(uint32_t addr)
{
	__disable_irq();
	
	IAPfunc(addr, 0, 0, 0x51);
	
	__enable_irq();
	
	CACHE_Invalid(addr);
}

/****************************************************************************************************************************************** 
* 函数名称:	FLASH_Write()
* 功能说明:	FLASH数据写入
* 输    入: uint32_t addr		数据要写入到Flash中的地址，字对齐
*			uint32_t buff[]		要写入Flash中的数据
*			uint32_t cnt		要写的数据的个数，以字为单位
* 输    出: 无
* 注意事项: 要写入的数据必须全部在同一页内，即addr/256 == (addr+cnt*4)/256
******************************************************************************************************************************************/
void FLASH_Write(uint32_t addr, uint32_t buff[], uint32_t cnt)
{	
	__disable_irq();
	
	IAPfunc(addr, (uint32_t)buff, cnt*4, 0x52);
	
	__enable_irq();
	
	CACHE_Invalid(addr);
}

/****************************************************************************************************************************************** 
* 函数名称:	FLASH_Read()
* 功能说明:	FLASH数据读取
* 输    入: uint32_t addr		要读取的数据在Flash中的地址，字对齐
*			uint32_t buff[]		读取到的数据存入buff指向的内存
*			uint32_t cnt		要读取的数据的个数，以字为单位
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void FLASH_Read(uint32_t addr, uint32_t buff[], uint32_t cnt)
{	
	uint32_t i;
	
	while(FLASH->STAT & FLASH_STAT_BUSY_Msk);
	
	FLASH->CR = (1 << FLASH_CR_FFCLR_Pos);			//Clear FIFO
	
	FLASH->CR = (FLASH_CMD_READ_DATA << FLASH_CR_CMD_Pos) |
				(0 << FLASH_CR_LEN_Pos);			//连续读
	FLASH->ADDR = addr;
	FLASH->START = 1;
	
	for(i = 0; i < cnt; i++)
	{
		while(FLASH->STAT & FLASH_STAT_FE_Msk);		//FIFO Empty
		
		buff[i] = FLASH->DATA;
	}
	
	FLASH->START = 0;
	
	FLASH->CR = (1 << FLASH_CR_FFCLR_Pos);			//Clear FIFO
	FLASH->CR = 0;
}
