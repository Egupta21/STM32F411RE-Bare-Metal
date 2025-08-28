/*
 * stm32f411xx.h
 *
 *  Created on: Jul 25, 2025
 *      Author: easha
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>
#define __vo volatile


/*
 * Processor Specific Details | ARM Cortex Mx NVIC ISERx register address
 */

#define NVIC_ISER0							((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1							((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2							((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3							((__vo uint32_t*) 0xE000E10C)

#define NVIC_ICER0							((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER1							((__vo uint32_t*) 0xE000E184)
#define NVIC_ICER2							((__vo uint32_t*) 0xE000E188)
#define NVIC_ICER3							((__vo uint32_t*) 0xE000E18C)

#define NVIC_IPR_BASE_ADDR					((__vo uint32_t*) 0xE000E400)

/*
 * IRQ mapping
 */

#define IRQ_NUM_EXTI0						6
#define IRQ_NUM_EXTI1						7
#define IRQ_NUM_EXTI2						8
#define IRQ_NUM_EXTI3						9
#define IRQ_NUM_EXTI4						10
#define IRQ_NUM_EXTI9_5						23
#define IRQ_NUM_EXTI15_10					40

/*
 * IRQ Priority
 */

#define NVIC_IRQ_PRIO0						0
#define NVIC_IRQ_PRIO1						1
#define NVIC_IRQ_PRIO2						2
#define NVIC_IRQ_PRIO3						3
#define NVIC_IRQ_PRIO4						4
#define NVIC_IRQ_PRIO5						5
#define NVIC_IRQ_PRIO6						6
#define NVIC_IRQ_PRIO7						7
#define NVIC_IRQ_PRIO8						8
#define NVIC_IRQ_PRIO9						9
#define NVIC_IRQ_PRIO10						10
#define NVIC_IRQ_PRIO11						11
#define NVIC_IRQ_PRIO12						12
#define NVIC_IRQ_PRIO13						13
#define NVIC_IRQ_PRIO14						14
#define NVIC_IRQ_PRIO15						15

/*
 * Base addresses for memory
 */
#define FLASH_BASEADDR						(uint32_t) 0x08000000  			// Base Address for Flash memory
#define SRAM1_BASEADDR						0x20000000U 					// Base address for SRAM1
//#define SRAM2_BASEADDR						0x20000000U					//
#define ROM									0X1FFF0000U						// Base address for ROM
#define SRAM1 								SRAM1_BASEADDR					// Defining SRAM 1

/*
 * Base addressed for busses
 */
#define PERIPH_BASEADDR						0x40000000U						// base address of peripherals
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					(PERIPH_BASEADDR + 0x10000)		// base address of APB2 peripherals
#define AHB1PERIPH_BASEADDR					(PERIPH_BASEADDR + 0x20000)		// base address of AHB1 peripherals
#define AHB2PERIPH_BASEADDR					(PERIPH_BASEADDR + 0x10000000)	// base addr or AHB2 peripherals

/*
 * Base addresses for RCC
 */

#define RCC_BASEADDR						0x40023800U
/*
 * Base addresses for GPIO PORTS
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)

/*
 * Base addresses for peripherals hanging on APB1 bus
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
//#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
//#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
//#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses for peripherals hanging on APB2 bus
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)

/*
 * Clock Enable and Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()						(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()						(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()						(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()						(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()						(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()						(RCC->AHB1ENR |= (1 << 7))
#define GPIOA_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 7))

#define GPIOA_REG_RESET()					do{(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()					do{(RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()					do{(RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()					do{(RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()					do{(RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()					do{(RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * Clock Enable and Disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()						(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()						(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()						(RCC->APB1ENR |= (1 << 23))
#define I2C1_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Enable and Disable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()						(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()						(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()						(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()						(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()						(RCC->APB2ENR |= (1 << 20))
#define SPI1_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 20))


/*
 * Clock Enable and Disable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()					(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()					(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()					(RCC->APB2ENR |= (1 << 5))
#define USART1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Enable and Disable macros for SYSCFGx peripherals
 */

#define SYSCFG_PCLK_EN()					(RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 14))

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t
 */

#define GPIOA								((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH								((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define SPI1								((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2								((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3								((SPI_RegDef_t*) SPI3_BASEADDR)

#define RCC									((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI								((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG								((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

/*
 * Generic Macros
 */

#define ENABLE								1
#define DISABLE								0
#define SET									ENABLE
#define RESET								DISABLE
#define HIGH								SET
#define LOW									DISABLE





/////////////////////////////////////// Register Definition Structures ///////////////////////////////////////////////
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];						// AFR[0] = AFRL and AFR[1] = AFRH
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t RES1;
	__vo uint32_t RES2;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RES3;
	__vo uint32_t RES4;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t RES5;
	__vo uint32_t RES6;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RES7;
	__vo uint32_t RES8;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t RES9;
	__vo uint32_t RES10;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RES11;
	__vo uint32_t RES12;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RES13;
	__vo uint32_t RES14;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t RES15;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RES[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

typedef struct
{
	uint32_t SPI_CR1;
	uint32_t SPI_CR2;
	uint32_t SPI_SR;
	uint32_t SPI_DR;
	uint32_t SPI_CRCPR;
	uint32_t SPI_RXCRCR;
	uint32_t SPI_TXCRCR;
	uint32_t SPI_I2SCFGR;
	uint32_t SPI_I2SPR;
}SPI_RegDef_t;
//RCC_RegDef_t *pRCC = RCC;
//GPIO_RegDef_t *pGPIOA = GPIOA;

void EXTI0_IRQHandler(void);

#endif /* INC_STM32F411XX_H_ */
