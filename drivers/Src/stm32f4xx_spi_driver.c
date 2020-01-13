/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: 11 ene. 2020
 *      Author: XaviR
 */

#include "stm32f4xx_spi_driver.h"
#include "stm32f446xx.h"


void SPI_PeriClockControl(SPI_RegDef_t* pSPIx,uint8_t Enable){

	if(Enable==ENABLE){

			if(pSPIx == SPI1)				SPI1_PCLCK_ENABLE();
			else if (pSPIx == SPI2)			SPI2_PCLCK_ENABLE();
			else if (pSPIx == SPI3)			SPI3_PCLCK_ENABLE();
			else if (pSPIx == SPI4)			SPI4_PCLCK_ENABLE();
		}
		else{

			}

}

/*Init and De-Init*/
void SPI_Init(SPI_Handle_t* pSPIHandle){

	//First lets configure the SPI_CR1 register.

	uint32_t tempreg = 0;

	// 1.Configure the device mode.
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// 2.Configure the bus config.

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX){

		// bidirectional mode be cleared.
		tempreg &= ~(1 << 15);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_HALF_DUPLEX){

		// bidirectional mode should be set.
		tempreg |= (1 << 15);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){

		// bidirectional mode should be cleared.
		tempreg &= ~(1 << 15);


		// RXONLY bit must be set.
		tempreg |= (1 << 10);

	}

	// 3.Configure the SPI Serial Clock Speed (BAUD RATE)
	tempreg |= pSPIHandle->SPIConfig.SPI_SCLKSpeed << 3;

	// 4.Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5.Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6.Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t* pSPIx){

}


/*Data Send and Receive*/
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer,uint32_t length){

}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer,uint32_t length){

}


/*IRQ Configuration and ISR handling*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t Enable){

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){

}

void SPI_IRQHandling(SPI_Handle_t* pHandle){

}
