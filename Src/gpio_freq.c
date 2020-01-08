/*
 * gpio_freq.c
 *
 *  Created on: 6 ene. 2020
 *      Author: XaviR
 */


#include "stm32f446xx.h"
#include "stm32f4xx_gpio_driver.h"
#include <stdio.h>
#include <string.h>

#define BTN_PRESS LOW


void ConfigPeripherals(GPIO_Handle_t* GpioLed,GPIO_Handle_t* GpioButton){

		GpioLed->pGPIOx  									= 	GPIOA;
		GpioLed->GPIO_PinConfig.GPIO_PinNumber 				= 	GPIO_PIN_NO_9;
		GpioLed->GPIO_PinConfig.GPIO_PinMode 				= 	GPIO_MODE_OUTPUT;
		GpioLed->GPIO_PinConfig.GPIO_PinSpeed 				= 	GPIO_SPEED_FAST ;
		GpioLed->GPIO_PinConfig.GPIO_PinOPType 				= 	GPIO_OP_TYPE_PP;
		GpioLed->GPIO_PinConfig.GPIO_PinPuPdControl			= 	GPIO_NO_PUPD;

		GPIO_PeriClockControl(GPIOA,ENABLE);
		GPIO_Init(GpioLed);

		GpioButton->pGPIOx 									=	GPIO_USER_BUTTON;
		GpioButton->GPIO_PinConfig.GPIO_PinNumber 			= 	BUTTON_USER_NUCLEO;
		GpioButton->GPIO_PinConfig.GPIO_PinMode 			= 	GPIO_MODE_IT_FT;
		GpioButton->GPIO_PinConfig.GPIO_PinSpeed 			= 	GPIO_SPEED_FAST;
		GpioButton->GPIO_PinConfig.GPIO_PinPuPdControl		= 	GPIO_PIN_PU;

		GPIO_PeriClockControl(GPIO_USER_BUTTON,ENABLE);
		GPIO_Init(GpioButton);

		//IRQ configurations
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10,NVIC_IRQ_PRI15);
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10,ENABLE);

}

void EXTI15_10_IRQHandler(void){

	GPIO_IRQHandling(BUTTON_USER_NUCLEO); //Clear the pending event from EXTI line.
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_9);
}

int main(void){

	GPIO_Handle_t GpioLed,GpioButton;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioButton,0,sizeof(GpioButton));

	ConfigPeripherals(&GpioLed, &GpioButton);

	while(1);
	//return 0;
}

