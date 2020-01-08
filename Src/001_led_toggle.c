/*
 * 001_led_toggle.c
 *
 *  Created on: 25 dic. 2019
 *      Author: Xavi
 */
#include "stm32f446xx.h"
#include "stm32f4xx_gpio_driver.h"
#include <stdio.h>

void delay(){

	for(uint32_t i=0;i<500000;i++);
}

int main(void){

	GPIO_Handle_t GPIO_Led,GPIO_button;

	GPIO_Led.pGPIOx  								= 	GPIO_USER_LED;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber 			= 	LED_USER_NUCLEO;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode 			= 	GPIO_MODE_OUTPUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed 			= 	GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType 			= 	GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl		= 	GPIO_NO_PUPD;

	GPIO_button.pGPIOx								=	GPIO_USER_BUTTON;
	GPIO_button.GPIO_PinConfig.GPIO_PinNumber		=	BUTTON_USER_NUCLEO;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode			=	GPIO_MODE_INPUT;
	GPIO_button.GPIO_PinConfig.GPIO_PinSpeed		=  	GPIO_SPEED_FAST;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIO_USER_LED, ENABLE);
	GPIO_PeriClockControl(GPIO_USER_BUTTON, ENABLE);

	GPIO_Init(&GPIO_Led);
	GPIO_Init(&GPIO_button);





	while(1){

		if(!GPIO_ReadFromInputPin(GPIO_USER_BUTTON,BUTTON_USER_NUCLEO)){
			delay();
			GPIO_ToggleOutputPin(GPIO_USER_LED,LED_USER_NUCLEO);
		}

	}

	return 0;


}


void EXTI0_IRQHandler(void){

	GPIO_IRQHandling(0);
}
