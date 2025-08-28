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

typedef struct
{
	SPI_RegDef_t *pSPIx; // base address of spi being used
	SPI_Config_t SPI_PinConfig;
}SPI_Handle_t;

void SPI_Init(SPI_Handle_t* pSPI_Handle);
void SPI_DeInit(SPI_Handle_t* pSPI_Handle);

uint16_t SPI_Tx(SPI_RegDef_t* SPI_RegDef, uint16_t message);
uint16_t SPI_Rx(SPI_RegDef_t* SPI_RegDef, uint16_t message);


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
