/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: 11 ene. 2020
 *      Author: XaviR
 */

#include "stm32f4xx_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t* pSPIx,uint8_t Enable){

}

/*Init and De-Init*/
void SPI_Init(SPI_Handle_t* pSPIHandle){

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
