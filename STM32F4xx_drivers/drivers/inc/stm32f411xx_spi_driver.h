/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Aug 27, 2025
 *      Author: easha
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include <stm32f411xx.h>

/*
 * @GPIO_PIN_MODES
 * possible gpio pin modes
 */
typedef struct
{
	uint8_t SPI_DevideMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF; // size of data
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_SclkSpeed;
}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t *pSPIx; // base address of spi being used
	SPI_Config_t SPI_PinConfig;
}SPI_Handle_t;

/*
 * Peripheral Clock Setup
 */

/*
 * SPI configuration macros
 */

#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLERX_RXONLY		4

#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

/*
 * Setup functions
 */
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_Handle_t *pSPI_Handle);

/*
 * Data Transfer
 */
uint16_t SPI_Tx(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t dataLen);
uint16_t SPI_Rx(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t dataLen);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle);
void SPI_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Extras
 */


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
