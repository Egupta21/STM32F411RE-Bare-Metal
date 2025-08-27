/*
 * stm32f411xx_exti_driver.h
 *
 *  Created on: Aug 8, 2025
 *      Author: easha
 */

#ifndef INC_STM32F411XX_EXTI_DRIVER_H_
#define INC_STM32F411XX_EXTI_DRIVER_H_

#include <stm32f411xx.h>

typedef struct
{
	uint8_t EXTI_PinNumber;
	uint8_t EXTI_InterruptMask;
	uint8_t EXTI_EventMask;
	uint8_t EXTI_RisingTrigger;
	uint8_t EXTI_FallingTrigger;
	uint8_t EXTI_SoftwareInterrupt;
	uint8_t EXTI_PendingBit;
}EXTI_PinConfig_t;

/*
 * Handler struct for EXTI
 */
typedef struct{
	EXTI_RegDef_t *pEXTIx;
	EXTI_PinConfig_t EXTI_PinConfig;
}EXTI_Handle_t;


/*
 * @EXTI_PIN_NUMBERS
 */
#define EXTI_PIN_0							0
#define EXTI_PIN_1							1
#define EXTI_PIN_2							2
#define EXTI_PIN_3							3
#define EXTI_PIN_4							4
#define EXTI_PIN_5							5
#define EXTI_PIN_6							6
#define EXTI_PIN_7							7
#define EXTI_PIN_8							8
#define EXTI_PIN_9							9
#define EXTI_PIN_10							10
#define EXTI_PIN_11							11
#define EXTI_PIN_12							12
#define EXTI_PIN_13							13
#define EXTI_PIN_14							14
#define EXTI_PIN_15							15
#define EXTI_PIN_16							16
#define EXTI_PIN_17							17
#define EXTI_PIN_18							18
#define EXTI_PIN_21							21
#define EXTI_PIN_22							22

/*
 * EXTI Pin Modes
 */

#define EXTI_IMR_MASKED						0
#define EXTI_IMR_UNMASKED					1

#define EXTI_EMR_MASKED						0
#define EXTI_EMR_UNMASKED					1

#define EXTI_RTSR_DIS						0
#define EXTI_RTSR_EN						1

#define EXTI_FTSR_DIS						0
#define EXTI_FTSR_EN						1

#define EXTI_SWIE_DIS						0
#define EXTI_SWIE_EN						1

#define EXTI_P_NOREQ						0
#define EXTI_P_REQ							1
#endif /* INC_STM32F411XX_EXTI_DRIVER_H_ */
