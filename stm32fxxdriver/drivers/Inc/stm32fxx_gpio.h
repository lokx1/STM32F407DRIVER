/*
 * stm32fxx_gpio.h
 *
 *  Created on: Jul 29, 2024
 *      Author: ADMIN
 */

#ifndef INC_STM32FXX_GPIO_H_
#define INC_STM32FXX_GPIO_H_

#include "stm32fxx.h"

#define NO_PR_BITS_IMPLEMENTED 	4



typedef struct{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;	//<GPIO_PIN_MODE>
	uint8_t GPIO_PinSpeed;		//<GPIO_PIN_SPEED>
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;
typedef struct{


	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;




}GPIOx_Handle_t;
/*
 * GPIO pin number
 */

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * GPIO Pin Modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4 	//input falling edges
#define GPIO_MODE_IT_RT		5	//input rising edges
#define GPIO_MODE_IT_RFT	6	//input rising edges falling edges trigger


/*
 * GPIO pins possible outputs types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * GPIO pins possible outputs speed
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * GPIO pins pull up/down
 */
#define GPIO_NO_PUPD 		0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*
 * API supported by this driver
 *
 */
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t*GPIO,uint8_t state);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIOx_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 * Data read and write
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t state);
void GPIO_IRQHandling(uint8_t pin);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);








#endif /* INC_STM32FXX_GPIO_H_ */
