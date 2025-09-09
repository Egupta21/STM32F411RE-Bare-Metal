/*
 * spitx.c
 *
 *  Created on: Sep 8, 2025
 *      Author: easha
 */

#include "stm32f411xx.h"
#include "stm32f411xx_spi_driver.h"

// SPI2 pins
// PB15 -- MOSI
// PB14 -- MISO
// PB13 -- SCK
// PB12 -- NSS
// Alt function mode : 5

int main(void)
{
	return 0;
	SPI_Handle_t *pspidevice;
	pspidevice->pSPIx = SPI2;

}
