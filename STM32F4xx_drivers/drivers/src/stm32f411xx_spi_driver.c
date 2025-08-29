/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Aug 27, 2025
 *      Author: easha
 */

#include <stm32f411xx_spi_driver.h>
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
