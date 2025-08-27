/*
 * 001LedToggle.c
 *
 *  Created on: Aug 6, 2025
 *      Author: easha
 */

#include <stm32f411xx_gpio_driver.h>

void delay(void)
{
	for(uint32_t i = 0; i < 250000; i++){
		i++;
	}
}
int main(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;

	// Push-Pull LED toggle Config
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NONE;

	// Open-Drain LED Toggle Config
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PU;

	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PU;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	while(1)
	{
		uint16_t val = GPIO_ReadFromInputPin(GpioButton.pGPIOx, GpioButton.GPIO_PinConfig.GPIO_PinNumber);
		if(val == LOW)
		{
			//delay();
			GPIO_WriteToOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber, 1);
		}
		else
		{
			//delay();
			GPIO_WriteToOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber, 0);
		}

		// flash led
		//GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		//delay();
	}


}

