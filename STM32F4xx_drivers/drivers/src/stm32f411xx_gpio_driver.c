/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jul 30, 2025
 *      Author: easha
 */


#include <stm32f411xx_gpio_driver.h>
#include <stm32f411xx_exti_driver.h>


/*
 * Peripheral Clock Setup
 */

/******************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables the clock for the given GPIO port
 *
 * @param[in]		- base address of GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else if(ENorDI == DISABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp = 0;
	// turn on the clock for the GPIO that is being setup
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// configure the mode of the gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x11 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits
		pGPIOHandle->pGPIOx->MODER |= temp; // setting bits
		temp = 0;
	}
	else{
		// interrupt mode

		//EXTI_PinConfig_t *pEXTI_PinConfig;
		//pEXTI_PinConfig->EXTI_FallingTrigger = EXTI_RTSR_EN
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// configure the RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// configure the FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();
		uint8_t extiRegIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t extiRegShift = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4 * 4;
		uint8_t val = GPIO_To_SysCfgEntry(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[extiRegIndex] &= ~(0xF << extiRegShift); // Clear the 4 bits
		SYSCFG->EXTICR[extiRegIndex] |=  (val << extiRegShift); // Set new value

		// enable the exti interrupt delivery using IMR
		// this unmasks the line corresponding to the pin
		// Ex. let PinNumber = 5, EXTI5 is now unmasked
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// configure the output type of the pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing bits
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// configure the speed of the gpio pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x11 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// configure the pupd of the pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x11 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// configure the alternate function mode of the pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
			pGPIOHandle->pGPIOx->AFR[0] &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits
			pGPIOHandle->pGPIOx->AFR[0] |= temp;
		}
		else{
			pGPIOHandle->pGPIOx->AFR[1] &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits
			pGPIOHandle->pGPIOx->AFR[1] |= temp;
		}
		temp = 0;
	}

}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}

/*
 * Misc
 */

uint8_t GPIO_To_SysCfgEntry(GPIO_RegDef_t* pGPIOx){
	uint8_t val;
	if(pGPIOx == GPIOA){
		val = 0b0000;
	}
	else if(pGPIOx == GPIOB){
		val = 0b0001;
	}
	else if(pGPIOx == GPIOC){
		val = 0b0010;
	}
	else if(pGPIOx == GPIOD){
		val = 0b0011;
	}
	else if(pGPIOx == GPIOE){
		val = 0b0100;
	}
	else if(pGPIOx == GPIOH){
		val = 0b0111;
	}
	return val;
}
/*
 * Data read and write
 */
uint16_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint16_t value = 0; // value to store return data
	value = (pGPIOx->IDR & (1 << PinNumber)); // store the value of the pin number in value
	return value >> PinNumber; // return value shifted to right

	/*
	 * Example scenario
	 *
	 * let PinNumber = 2
	 * let pGPIOx = 1 (high)
	 *
	 * pGPIOx->IDR & (1 << 2(PinNumber) evaluates to 0b0100 & 0b0100, which is 1
	 * value is now 1
	 * return value >> PinNumber, so 0b0100 >> 2 which is 0b0001, hence return 1
	 */
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == SET) // if value is high
	{
		(pGPIOx->ODR |= (1 << PinNumber)); // set pin for that gpio to high
	}
	else if(Value == DISABLE) // if value is low
	{
		(pGPIOx->ODR &= ~(1 << PinNumber)); // set pin for that gpio to low
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value; // value can be something like 0b010011101...
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);

	/*if((pGPIOx->ODR & (1 << PinNumber)) > 0)
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}*/
}
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		if(IRQNumber <= 31)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else{
		if(IRQNumber <= 31)
		{
			// program ICER0 register
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			// program ICER1 register
			*NVIC_ICER1 &= ~(1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			// program ICER2 register
			*NVIC_ICER2 &= ~(1 << (IRQNumber % 64));
		}
	}
}

void GPIO_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// calculate ipr register to set
	uint8_t prRegVal = IRQNumber/4;
	uint8_t prRegShift = IRQNumber%4 * 8;

	*(NVIC_IPR_BASE_ADDR + prRegVal) |= (IRQPriority << (prRegShift + 4));
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the exti pr register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
