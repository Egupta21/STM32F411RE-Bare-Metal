/*
 * button_interrupt.c
 *
 *  Created on: Aug 18, 2025
 *      Author: easha
 */

#include <stm32f411xx_gpio_driver.h>
#include <string.h>

void delay(void)
{
	for(uint32_t i = 0; i < 250000; i++){
	}
}

int main(void)
{
	//GPIO_RegDef_t* pGPIOD_RegDef = GPIOH;
	// setup gpio to be input and mode to be pin 0
	GPIO_Handle_t GPIOButton;
	memset(&GPIOButton, 0, sizeof(GPIOButton));
	GPIOButton.pGPIOx = GPIOC;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PU;

	GPIO_Handle_t GPIOLed;
	memset(&GPIOLed, 0, sizeof(GPIOLed));
	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;

	GPIO_Init(&GPIOButton);
	GPIO_Init(&GPIOLed);

	GPIO_PriorityConfig(IRQ_NUM_EXTI3, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI3, ENABLE);

	while(1);
/*	while(1)
	{
		uint16_t res = GPIO_ReadFromInputPin(GPIOButton.pGPIOx, GPIOButton.GPIO_PinConfig.GPIO_PinNumber);
		if(res == LOW)
		{
			GPIO_WriteToOutputPin(GPIOLed.pGPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber, 0);
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOLed.pGPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber, 1);
		}
	}*/

}

void EXTI3_IRQHandler(void)
{
	delay(); // 200 ms
	GPIO_IRQHandling(GPIO_PIN_3);
	//GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_6, 1);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_6);
}
