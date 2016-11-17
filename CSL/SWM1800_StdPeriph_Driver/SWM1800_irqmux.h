#ifndef __SWM1800_IRQMUX_H__
#define __SWM1800_IRQMUX_H__


void IRQ_Connect(uint32_t periph_interrupt, uint32_t IRQn, uint32_t priority);	//将外设中断连接到内核中断IRQ0--IRQ31

uint32_t IRQ_Which(uint32_t IRQn);	//IRQ16--IRQ31每一个IRQ上可以连接两个外设中断，此函数用于判断两个外设中断中的哪一个触发了中断请求


#define IRQ_INT0	0x000
#define IRQ_INT1	0x100

#define IRQ_BOTH	0x101
#define IRQ_NONE	0x102


#define IRQ0_15_GPIOA0		0x00
#define IRQ0_15_GPIOA1		0x01
#define IRQ0_15_GPIOA2		0x02
#define IRQ0_15_GPIOA3		0x03
#define IRQ0_15_GPIOA4		0x04
#define IRQ0_15_GPIOA5		0x05
#define IRQ0_15_GPIOA6		0x06
#define IRQ0_15_GPIOA7		0x07
#define IRQ0_15_GPIOA8		0x08
#define IRQ0_15_GPIOA9		0x09
#define IRQ0_15_GPIOA10	    0x0A
#define IRQ0_15_GPIOA11	    0x0B
#define IRQ0_15_GPIOA12	    0x0C
#define IRQ0_15_GPIOA13	    0x0D
#define IRQ0_15_GPIOA14	    0x0E
#define IRQ0_15_GPIOA15	    0x0F

#define IRQ0_15_GPIOB0		0x10
#define IRQ0_15_GPIOB1		0x11
#define IRQ0_15_GPIOB2		0x12
#define IRQ0_15_GPIOB3		0x13
#define IRQ0_15_GPIOB4		0x14
#define IRQ0_15_GPIOB5		0x15
#define IRQ0_15_GPIOB6		0x16
#define IRQ0_15_GPIOB7		0x17
#define IRQ0_15_GPIOB8		0x18
#define IRQ0_15_GPIOB9		0x19
#define IRQ0_15_GPIOB10	    0x1A
#define IRQ0_15_GPIOB11	    0x1B
#define IRQ0_15_GPIOB12	    0x1C
#define IRQ0_15_GPIOB13	    0x1D
#define IRQ0_15_GPIOB14	    0x1E
#define IRQ0_15_GPIOB15	    0x1F

#define IRQ0_15_GPIOC0		0x20
#define IRQ0_15_GPIOC1		0x21
#define IRQ0_15_GPIOC2		0x22
#define IRQ0_15_GPIOC3		0x23
#define IRQ0_15_GPIOC4		0x24
#define IRQ0_15_GPIOC5		0x25
#define IRQ0_15_GPIOC6		0x26
#define IRQ0_15_GPIOC7		0x27

#define IRQ0_15_GPIOD0		0x30
#define IRQ0_15_GPIOD1		0x31
#define IRQ0_15_GPIOD2		0x32
#define IRQ0_15_GPIOD3		0x33
#define IRQ0_15_GPIOD4		0x34
#define IRQ0_15_GPIOD5		0x35
#define IRQ0_15_GPIOD6		0x36
#define IRQ0_15_GPIOD7		0x37

#define IRQ0_15_GPIOE0		0x38
#define IRQ0_15_GPIOE1		0x39
#define IRQ0_15_GPIOE2		0x3A
#define IRQ0_15_GPIOE3		0x3B
#define IRQ0_15_GPIOE4		0x3C
#define IRQ0_15_GPIOE5		0x3D
#define IRQ0_15_GPIOE6		0x3E
#define IRQ0_15_GPIOE7		0x3F

#define IRQ0_15_SPI1		0x50
#define IRQ0_15_CORDIC		0x51
#define IRQ0_15_SPI0		0x53
#define IRQ0_15_TIMR		0x54
#define IRQ0_15_UART0		0x58
#define IRQ0_15_UART1		0x59
#define IRQ0_15_UART2		0x5A
#define IRQ0_15_PWM			0x5B
#define IRQ0_15_I2C0		0x5C
#define IRQ0_15_I2C1		0x5D
#define IRQ0_15_WDT			0x5E
#define IRQ0_15_ADC			0x5F
#define IRQ0_15_BOD			0x60
#define IRQ0_15_UART3		0x61
#define IRQ0_15_DMA			0x63
#define IRQ0_15_CACHE		0x64
#define IRQ0_15_FLASH		0x65
#define IRQ0_15_CAN			0x66
#define IRQ0_15_CMP			0x67
#define IRQ0_15_SDADC		0x68
#define IRQ0_15_HALL		0x69


#define IRQ16_31_GPIOA		0x00
#define IRQ16_31_GPIOB		0x01
#define IRQ16_31_GPIOC		0x02
#define IRQ16_31_GPIOD		0x03
#define IRQ16_31_GPIOE		0x04

#define IRQ16_31_SPI1		0x06
#define IRQ16_31_CORDIC		0x07
#define IRQ16_31_SPI0		0x09
#define IRQ16_31_TIMR		0x0A
#define IRQ16_31_HALL		0x0D
#define IRQ16_31_UART0		0x0E
#define IRQ16_31_UART1		0x0F
#define IRQ16_31_UART2		0x10
#define IRQ16_31_PWM		0x11
#define IRQ16_31_I2C0		0x12
#define IRQ16_31_I2C1		0x13
#define IRQ16_31_WDT		0x14
#define IRQ16_31_ADC		0x15
#define IRQ16_31_BOD		0x16
#define IRQ16_31_UART3		0x17
#define IRQ16_31_DMA		0x19
#define IRQ16_31_CACHE		0x1A
#define IRQ16_31_FLASH		0x1B
#define IRQ16_31_CAN		0x1C
#define IRQ16_31_CMP		0x1D
#define IRQ16_31_SDADC		0x1E




#endif //__SWM1800_IRQMUX_H__
