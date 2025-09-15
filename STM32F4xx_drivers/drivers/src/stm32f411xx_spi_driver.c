/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Aug 27, 2025
 *      Author: easha
 */

#include <stm32f411xx_spi_driver.h>

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*
 * Setup functions
 */
void SPI_Init(SPI_Handle_t *pSPI_Handle)
{
	SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);

	// set the spi mode
	pSPI_Handle->pSPIx->SPI_CR1 |= (pSPI_Handle->SPI_PinConfig.SPI_DevideMode << 2);

	// set BIDI/RXOnly for SPI Handler
	if(pSPI_Handle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// set bidi to 0
		pSPI_Handle->pSPIx->SPI_CR1 &= ~(1 << 15);
	}
	else if(pSPI_Handle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// set bidi to 1
		pSPI_Handle->pSPIx->SPI_CR1 |= (1 << 15);
	}
	else if(pSPI_Handle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLERX_RXONLY)
	{
		// clear bidi
		// set rx only bit
		pSPI_Handle->pSPIx->SPI_CR1 &= ~(1 << 15);
		pSPI_Handle->pSPIx->SPI_CR1 |= (1 << 10);
	}

	// set the DFF
	pSPI_Handle->pSPIx->SPI_CR1 |= (pSPI_Handle->SPI_PinConfig.SPI_DFF << 11);

	// set the cpol and cpha
	pSPI_Handle->pSPIx->SPI_CR1 |= (pSPI_Handle->SPI_PinConfig.SPI_CPOL << 1);
	pSPI_Handle->pSPIx->SPI_CR1 |= (pSPI_Handle->SPI_PinConfig.SPI_CPHA << 0);

	// configure the ssm
	pSPI_Handle->pSPIx->SPI_CR1 |= (pSPI_Handle->SPI_PinConfig.SPI_SSM << 9);
}

void SPI_DeInit(SPI_Handle_t *pSPI_Handle);

/*
 * Data Transfer
 */
void SPI_Tx(SPI_RegDef_t *SPI_RegDef, uint8_t *pTxBuffer, uint32_t dataLen)
{
	while(dataLen > 0)
	{
		while((SPI_RegDef->SPI_SR & (1 << 1)) == 0); // wait till TXE buffer is not empty

		if(SPI_RegDef->SPI_CR1 & (1 << 11)) // If dff is 16 bit
		{
			SPI_RegDef->SPI_DR |= *((uint16_t*)pTxBuffer);
			dataLen--;
			dataLen--;
			pTxBuffer += 2;
		}
		else
		{
			// data is 8 bit
			SPI_RegDef->SPI_DR |= *pTxBuffer;
			dataLen--;
			pTxBuffer++;
		}
	}
}

uint16_t SPI_Rx(SPI_RegDef_t *SPI_RegDef, uint8_t *pRxBuffer, uint32_t dataLen);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle);
void SPI_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
