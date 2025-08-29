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
	uint8_t GPIO_BusConfig;
	uint8_t GPIO_DFF;
	uint8_t GPIO_CPHA;
	uint8_t GPIO_CPOL;
	uint8_t GPIO_SSM;
	uint8_t GPIO_SclkSpeed;
}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t *pSPIx; // base address of spi being used
	SPI_Config_t SPI_PinConfig;
}SPI_Handle_t;

/*
 * Setup functions
 */
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_Handle_t *pSPI_Handle);

/*
 * Data Transfer
 */
uint16_t SPI_Tx(SPI_RegDef_t *SPI_RegDef, uint8_t *pTxBuffer, uint32_t dataLen);
uint16_t SPI_Rx(SPI_RegDef_t *SPI_RegDef, uint8_t *pRxBuffer, uint32_t dataLen);

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
