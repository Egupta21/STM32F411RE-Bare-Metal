/*
 * spitx.c
 *
 *  Created on: Sep 8, 2025
 *      Author: easha
 */

#include "stm32f411xx.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_gpio_driver.h"
#include <string.h>

// SPI2 pins
// PB15 -- MOSI
// PB14 -- MISO
// PB13 -- SCK
// PB12 -- NSS
// Alt function mode : 5

int main(void)
{

	char data[] = "Hello World";
	//return 0;
	GPIO_Handle_t SPIdevicePins;

	SPIdevicePins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIdevicePins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	SPIdevicePins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	SPIdevicePins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIdevicePins.pGPIOx = GPIOB;


	// Setup NSS
	SPIdevicePins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIdevicePins);

	// Setup SCK


	// Setup MISO


	// Setup MOSI
	SPIdevicePins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIdevicePins);

	// Setup SPI
	SPI_Handle_t SPIHandle;
	SPIHandle.SPI_PinConfig.SPI_CPHA = 0;
	SPIHandle.SPI_PinConfig.SPI_CPOL = 0;
	SPIHandle.SPI_PinConfig.SPI_DevideMode = SPI_DEVICE_MODE_MASTER;
	SPIHandle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIHandle.SPI_PinConfig.SPI_DFF = SPI_DFF_16BITS;
	SPIHandle.SPI_PinConfig.SPI_SSM = SPI_SSM_EN;
	SPIHandle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPIHandle.pSPIx = SPI2;

	SPI_Init(&SPIHandle);

	SPI_Tx(SPIHandle.pSPIx,(uint8_t*) data, strlen(data));



}
