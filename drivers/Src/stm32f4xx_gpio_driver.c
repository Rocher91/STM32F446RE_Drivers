/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: 15 dic. 2019
 *      Author: Xavi
 */


#include "stm32f4xx_gpio_driver.h"

/************************************************************
 * @fn		- GPIO_PeriClockControl
 * @brief	- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- pGPIOx 	*base addres of the gpio peripheral*
 * @param[in]  	- Enable 	*ENABLE or DISABLE macros*
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none

 ************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx,uint8_t Enable){

	if(Enable==ENABLE){

		if(pGPIOx == GPIOA)				GPIOA_PCLCK_ENABLE();
		else if (pGPIOx == GPIOB)		GPIOB_PCLCK_ENABLE();
		else if (pGPIOx == GPIOC)		GPIOC_PCLCK_ENABLE();
		else if (pGPIOx == GPIOD)		GPIOD_PCLCK_ENABLE();
		else if (pGPIOx == GPIOE)		GPIOE_PCLCK_ENABLE();
		else if (pGPIOx == GPIOF)		GPIOF_PCLCK_ENABLE();
		else if (pGPIOx == GPIOH)		GPIOH_PCLCK_ENABLE();
	}
	else{

	}
}


/************************************************************
 * @fn		- GPIO_Init
 * @brief	-
 *
 * @param[in]	-	pGPIOHandle 	* *
 * @param[in]  	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-

 ************************************************************/

void GPIO_Init(GPIO_Handle_t* pGPIOHandle){

	uint32_t temp=0;
	//Configure the mode

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG ){

		//the none interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;

	}else{

		//interrupt mode

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

			//	1.Configure the FTSR.
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit.
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){

			//	1.Configure the RTSR.
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit.
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){

			//	1.Configure the FTSR and RTSR.
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//	2.Configure the GPIO port selection in SYSCFG_EXTICR

		uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint32_t portcode =GPIOA_BASEADDR_TO_CODE( pGPIOHandle->pGPIOx);
		SYSCFG_PCLCK_ENABLE();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);

		// 	3.Enable the EXTI interrupt delivery using IMR.
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//configure the speed
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	//configure pupd settings
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//configure the optype
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTEN){

		uint32_t temp1=0,temp2=0;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF <<(4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4*temp2);
	}
}

/************************************************************
 * @fn		- GPIO_DeInit
 * @brief	-
 *
 * @param[in]	-	pGPIOx	* *
 * @param[in]  	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-

 ************************************************************/

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx){

	if(pGPIOx == GPIOA)				GPIOA_PCLCK_DISABLE();
	else if (pGPIOx == GPIOB)		GPIOB_PCLCK_DISABLE();
	else if (pGPIOx == GPIOC)		GPIOC_PCLCK_DISABLE();
	else if (pGPIOx == GPIOD)		GPIOD_PCLCK_DISABLE();
	else if (pGPIOx == GPIOE)		GPIOE_PCLCK_DISABLE();
	else if (pGPIOx == GPIOF)		GPIOF_PCLCK_DISABLE();
	else if (pGPIOx == GPIOH)		GPIOH_PCLCK_DISABLE();
}

/************************************************************
 * @fn		- GPIO_ReadFromInputPin
 * @brief	- Data read and write
 *
 * @param[in]	-	pGPIOx 		**
 * @param[in]  	-	PinNumber	**
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-

 ************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx,uint8_t PinNumber){

	uint8_t value = (uint8_t) (( pGPIOx->IDR >> PinNumber) & 0x00000001) ;
	return value;
}

/************************************************************
 * @fn		- GPIO_ReadFromInputPort
 * @brief	- Data read and write
 *
 * @param[in]	-	pGPIOx	*pGPIOx*
 * @param[in]  	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-

 ************************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx){

	uint16_t value = (uint16_t) pGPIOx->IDR;
	return value;
}

/************************************************************
 * @fn		- GPIO_WriteToOutputPin
 * @brief	-
 *
 * @param[in]	-	pGPIOx			* *
 * @param[in]  	-	PinNumber		* *
 * @param[in]	-	value			* *
 *
 * @return		-
 *
 * @Note		-

 ************************************************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx,uint8_t PinNumber,uint8_t value){

	if(value == GPIO_PIN_SET)			pGPIOx->ODR |= (1 << PinNumber);
	else if(value == GPIO_PIN_RESET)	pGPIOx->ODR &= ~(1 << PinNumber);
}

/************************************************************
 * @fn		- GPIO_WriteToOutputPort
 * @brief	-
 *
 * @param[in]	-	pGPIOx		* *
 * @param[in]  	-	value		* *
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-

 ************************************************************/

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx,uint16_t value){

	pGPIOx->ODR |= value;
}

/************************************************************
 * @fn		- GPIO_ToggleOutputPin
 * @brief	-
 *
 * @param[in]	-		pGPIOx		* *
 * @param[in]  	-		PinNumber	* *
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-

 ************************************************************/

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx,uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);
}

/************************************************************
 * @fn		- GPIO_IRQInterruptConfig
 * @brief	- IRQ Configuration
 *
 * @param[in]	-		IRQNumber		* *
 * @param[in]	-		Enable			* *
 *
 * @return		-
 *
 * @Note		-

 ************************************************************/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t Enable){

	if(Enable == ENABLE){

		if(IRQNumber <= 31){

			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}
		else if(IRQNumber > 31 && IRQNumber < 64){

			//Program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){

			//Program ISER2 register	64 to 96
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}

	}
	else{

		if(IRQNumber <= 31){

			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}

		else if(IRQNumber > 31 && IRQNumber < 64){

			//Program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}

		else if(IRQNumber >= 64 && IRQNumber < 96){

			//Program ICER2 register  64 to 96
			*NVIC_ICER1 |= (1 << IRQNumber % 64);
		}
	}
}

/************************************************************
 * @fn		- GPIO_IRQPriorityConfig
 * @brief	- IRQ Configuration
 *
 * @param[in]	-		IRQPriority		* *
 *
 * @return		-
 *
 * @Note		-

 *
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){

	//	1.First lets find out the ipr register
	uint8_t iprx =IRQNumber/4;
	uint8_t iprx_section =IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section)+(8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR  +  iprx ) |= ( IRQPriority << shift_amount );
}

/************************************************************
 * @fn		- GPIO_ToggleOutputPin
 * @brief	- ISR handling
 *
 * @param[in]	-		PinNumber		* *
 * @param[in]  	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-

 ************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber){

	//Clear the EXTI PR register corresponding to the pin number

	if(EXTI->PR & (1 <<PinNumber))
		EXTI->PR |= ( 1 <<PinNumber );	// Clear

}
