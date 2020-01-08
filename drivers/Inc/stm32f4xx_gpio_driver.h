/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: 15 dic. 2019
 *      Author: Xavi Rocher
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

//**********************************
//		@GPIO_PIN_MODES
//		GPIO pin possible modes

#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTEN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

//**********************************
//		@GPIO_PIN_NUMBERS
//		GPIO number pins possible

#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15

//**********************************
//		@GPIO_SPEED_MODES
//		GPIO pin possible output types

#define GPIO_OP_TYPE_PP		0		//Push-pull
#define GPIO_OP_TYPE_OD		1		//Open-drain


//**********************************
//		@GPIO_SPEED_MODES
//		GPIO pin possible output speeds

#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3


//**********************************
//GPIO pin pull up pull down configuration macros

#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

//**********************************


typedef struct{

	uint8_t GPIO_PinNumber;			/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t	GPIO_PinMode;  			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t	GPIO_PinSpeed;			/*!< possible values from @GPIO_SPEED_MODES >*/
	uint8_t	GPIO_PinPuPdControl;	/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t	GPIO_PinOPType;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t	GPIO_PinAltFunMode;		/*!< possible values from @GPIO_PIN_MODES >*/

}GPIO_PinConfig_t;

typedef struct{

	GPIO_RegDef_t* pGPIOx;  // this hold the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


// Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx,uint8_t Enable);


// Init and De-Init Functions
void GPIO_Init(GPIO_Handle_t* pGPIOPHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx,uint8_t PinNumber);

//IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t Enable);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
