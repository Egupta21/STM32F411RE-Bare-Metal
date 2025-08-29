/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Jul 30, 2025
 *      Author: easha
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include <stm32f411xx.h>

/*
 * @GPIO_PIN_MODES
 * possible gpio pin modes
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * Handler Struct for GPIO pin
 */

typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_0							0
#define GPIO_PIN_1							1
#define GPIO_PIN_2							2
#define GPIO_PIN_3							3
#define GPIO_PIN_4							4
#define GPIO_PIN_5							5
#define GPIO_PIN_6							6
#define GPIO_PIN_7							7
#define GPIO_PIN_8							8
#define GPIO_PIN_9							9
#define GPIO_PIN_10							10
#define GPIO_PIN_11							11
#define GPIO_PIN_12							12
#define GPIO_PIN_13							13
#define GPIO_PIN_14							14
#define GPIO_PIN_15							15


/*
 * @GPIO_PIN_MODES
 * Pin Modes
 */

#define GPIO_MODE_INPUT						0
#define GPIO_MODE_OUTPUT					1
#define GPIO_MODE_ALTFN						2
#define GPIO_MODE_ANALOG					3
#define GPIO_MODE_IT_FT						4
#define GPIO_MODE_IT_RT						5
#define GPIO_MODE_IT_RFT					6

#define GPIO_OTYPE_PP						0
#define GPIO_OTYPE_OD						1

#define GPIO_SPEED_LOW						0
#define GPIO_SPEED_MEDIUM					1
#define GPIO_SPEED_FAST						2
#define GPIO_SPEED_HIGH						3

#define GPIO_PUPDR_NONE						0
#define GPIO_PUPDR_PU						1
#define GPIO_PUPDR_PD						2
#define GPIO_PUPDR_RES						3
#define GPIO_AFR

/*
 * Peripheral Clock Setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

/*
 * Init and De-Init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint16_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Misc
 */
uint8_t GPIO_To_SysCfgEntry(GPIO_RegDef_t *pGPIOx);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
