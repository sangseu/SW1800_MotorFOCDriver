#ifndef __SWM1800_FLASH_H__
#define __SWM1800_FLASH_H__


void FLASH_Erase(uint32_t addr);
void FLASH_Write(uint32_t addr, uint32_t buff[], uint32_t cnt);
void FLASH_Read(uint32_t addr, uint32_t buff[], uint32_t cnt);


#define FLASH_CMD_READ_Identi   0x00
#define FLASH_CMD_READ_ID       0x01
#define FLASH_CMD_READ_STATL    0x02
#define FLASH_CMD_READ_STATH    0x03
#define FLASH_CMD_READ_DATA     0x04
#define FLASH_CMD_WRITE_STAT    0x08
#define FLASH_CMD_WRITE_PAGE    0x09
#define FLASH_CMD_ERASE_SECT    0x0A
#define FLASH_CMD_ERASE_BLK32K  0x0B
#define FLASH_CMD_ERASE_BLK64K  0x0C
#define FLASH_CMD_ERASE_CHIP    0x0D
#define FLASH_CMD_WRITE_SUSP    0x0E
#define FLASH_CMD_WRITE_RESUME  0x0F
#define FLASH_CMD_SLEEP         0x10
#define FLASH_CMD_WAKEUP        0x11
#define FLASH_CMD_WRITE_EN      0x12
#define FLASH_CMD_WRITE_DIS     0x13
#define FLASH_CMD_WRITE_SR_EN   0x14


#endif //__SWM1800_FLASH_H__
