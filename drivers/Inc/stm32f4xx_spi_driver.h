/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: 11 ene. 2020
 *      Author: XaviR
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

typedef struct{

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct{

	SPI_RegDef_t* pSPIx;  // this hold the base address of the SPI
	SPI_Config_t SPIConfig;

}SPI_Handle_t;


/* Peripheral Clock Setup*/
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx,uint8_t Enable);

/*Init and De-Init*/
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);


/*Data Send and Receive*/
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer,uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer,uint32_t length);


/*IRQ Configuration and ISR handling*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t Enable);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* pHandle);


#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
