/*
 * stm32f446xx.h
 *
 *  Created on: Dec 7, 2019
 *      Author: Xavi Rocher
 */

#include <stdint.h>

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define ROM					0x1FFF0000U
#define	ENABLE				1
#define	DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define HIGH				1
#define LOW					0


#define SRAM				SRAM1_BASEADDR

/*	AHBx AND APBx Bus Peripheral base addresses
 * */

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U
#define AHB3PERIPH_BASE		0xA0001000U

#define NVIC_PR_BASE_ADDR 			((volatile uint32_t*) 0xE000E400)
#define NO_PR_BITS_IMPLEMENTED		4

/****************START:Processor Specific Details ***
 *
 * 	ARM Cortex Mx Processor NVIC ISERx register Addresses
 *
 * */
#define NVIC_ISER0			(volatile uint32_t*) 0xE000E100
#define NVIC_ISER1			(volatile uint32_t*) 0xE000E104
#define NVIC_ISER2			(volatile uint32_t*) 0xE000E108
#define NVIC_ISER3			(volatile uint32_t*) 0xE000E10C


//	ARM Cortex Mx Processor NVIC ICERx register Addresses

#define NVIC_ICER0			(volatile uint32_t*) 0xE000E180
#define NVIC_ICER1			(volatile uint32_t*) 0xE000E184
#define NVIC_ICER2			(volatile uint32_t*) 0xE000E188
#define NVIC_ICER3			(volatile uint32_t*) 0xE000E18C


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 *
 * */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00)

#define CRC_BASEADDR		(AHB1PERIPH_BASE + 0x3000)
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)
#define FLASH_INT_BASEADDR	(AHB1PERIPH_BASE + 0x3C00)
#define BKPSRAM_BASEADDR	(AHB1PERIPH_BASE + 0x4000)

#define DMA1_BASEADDR		(AHB1PERIPH_BASE + 0x6000)
#define DMA2_BASEADDR		(AHB1PERIPH_BASE + 0x6400)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 *
 * */

#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00)

#define CAN1_BASEADDR		(APB1PERIPH_BASE + 0x6400)
#define CAN2_BASEADDR		(APB1PERIPH_BASE + 0x6800)

#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)

#define TIM2_BASEADDR		(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR		(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR		(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR		(APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR		(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR		(APB1PERIPH_BASE + 0x1400)
#define TIM12_BASEADDR		(APB1PERIPH_BASE + 0x1800)
#define TIM13_BASEADDR		(APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASEADDR		(APB1PERIPH_BASE + 0x2000)

#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)


#define RTC_BKP_BASEADDR	(APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR		(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR		(APB1PERIPH_BASE + 0x3000)

#define DAC_BASEADDR		(APB1PERIPH_BASE + 0x7400)
#define PWR_BASEADDR		(APB1PERIPH_BASE + 0x7000)



/*
 * Base addresses of peripherals which are hanging on APB2 bus
 *
 * */

#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00)
#define SYSFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)

#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR		(APB2PERIPH_BASE + 0x3400)
#define SDMMC_BASEADDR		(APB2PERIPH_BASE + 0x2C00)

#define ADC_BASEADDR		(APB2PERIPH_BASE + 0x2000)

#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)

#define TIM8_BASEADDR		(APB2PERIPH_BASE + 0x0400)
#define TIM9_BASEADDR		(APB2PERIPH_BASE + 0x4000)
#define TIM10_BASEADDR		(APB2PERIPH_BASE + 0x4400)
#define TIM11_BASEADDR		(APB2PERIPH_BASE + 0x4800)
#define TIM1_BASEADDR		(APB2PERIPH_BASE + 0x0000)

#define SAI1_BASEADDR		(APB2PERIPH_BASE + 0x5800)
#define SAI2_BASEADDR		(APB2PERIPH_BASE + 0x5C00)




/*********************** Peripheral Register Definition Structures ***********************/


typedef struct{

	volatile uint32_t MODER;  		/*	 Offset: 0x00	Description: 	*/
	volatile uint32_t OTYPER;		/*	 Offset: 0x04	Description: 	*/
	volatile uint32_t OSPEEDER;		/*	 Offset: 0x08	Description: 	*/
	volatile uint32_t PUPDR;		/*	 Offset: 0x0C	Description: 	*/
	volatile uint32_t IDR;			/*	 Offset: 0x10	Description: 	*/
	volatile uint32_t ODR;			/*	 Offset: 0x14	Description: 	*/
	volatile uint32_t BSRR;			/*	 Offset: 0x18	Description: 	*/
	volatile uint32_t LCKR;			/*	 Offset: 0x1C	Description: 	*/
	volatile uint32_t AFR[2];		/*	 Offset: 0x20	Description: 	*/

}GPIO_RegDef_t;

/*********************** SPI Register Definition Structures ***********************/

typedef struct{

	volatile uint32_t CR1;			/*	 Offset: 0x00	Description: 	*/
	volatile uint32_t CR2;			/*	 Offset: 0x04	Description: 	*/
	volatile uint32_t SR;			/*	 Offset: 0x08	Description: 	*/
	volatile uint32_t DR;			/*	 Offset: 0x0C	Description: 	*/
	volatile uint32_t CRCPR;		/*	 Offset: 0x10	Description: 	*/
	volatile uint32_t RXCPR;		/*	 Offset: 0x14	Description: 	*/
	volatile uint32_t TXCPR;		/*	 Offset: 0x18	Description: 	*/
	volatile uint32_t I2SCFGR;		/*	 Offset: 0x1C	Description: 	*/
	volatile uint32_t I2SPR;		/*	 Offset: 0x20	Description: 	*/

}SPI_RegDef_t;

#define SPI1 ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t*) SPI4_BASEADDR)

/*********************** Bit position definitions of SPI Peripheral ***********************/

// Bit position definitions SPI_CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSB_FIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RX_ONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRC_NEXT		12
#define SPI_CR1_CRC_EN			13
#define SPI_CR1_BIDI_OE			14
#define SPI_CR1_BIDI_MODE		15

// Bit position definitions SPI_CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7


// Bit position definitions SPI_SR
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8


// Bit position definitions SPI_I2SCFGR
#define SPI_I2SCFGR_CHLEN		0
#define SPI_I2SCFGR_DATLEN		1
#define SPI_I2SCFGR_CKPOL		3
#define SPI_I2SCFGR_I2SSTD		4
#define SPI_I2SCFGR_RES			6
#define SPI_I2SCFGR_PCMSYNC		7
#define SPI_I2SCFGR_I2SCFG		8
#define SPI_I2SCFGR_I2SE		10
#define SPI_I2SCFGR_I2SMOD		11
#define SPI_I2SCFGR_ASTREN		12

// Bit position definitions SPI_I2SSPR
#define SPI_I2SSPR_I2SDIV		0
#define SPI_I2SSPR_ODD			8
#define SPI_I2SSPR_MCKOE		9

/******************************************************************************************/


/*********************** EXTI Register Definition Structures ***********************/


typedef struct{

	volatile uint32_t IMR;  		/*	 Offset: 0x00	Description: 	*/
	volatile uint32_t EMR;			/*	 Offset: 0x04	Description: 	*/
	volatile uint32_t RTSR;			/*	 Offset: 0x08	Description: 	*/
	volatile uint32_t FTSR;			/*	 Offset: 0x0C	Description: 	*/
	volatile uint32_t SWIER;		/*	 Offset: 0x10	Description: 	*/
	volatile uint32_t PR;			/*	 Offset: 0x14	Description: 	*/

}EXTI_RegDef_t;


#define EXTI ((EXTI_RegDef_t*) EXTI_BASEADDR)

/*********************** SYSCFG Register Definition Structures ***********************/


typedef struct{

	volatile uint32_t MEMRMP;  				/*	 Offset: 0x00				Description: 	*/
	volatile uint32_t PMC;					/*	 Offset: 0x04				Description: 	*/
	volatile uint32_t EXTICR[4];			/*	 Offset: 0x08	-	0x14	Description: 	*/
	volatile uint32_t RESERVED1[2];			/*	 Offset: 0x18	-	0x1C	Description: 	*/
	volatile uint32_t CMPCR;				/*	 Offset: 0x20				Description: 	*/
	volatile uint32_t RESERVED2[2];			/*	 Offset: 0x24	-	0x28	Description: 	*/
	volatile uint32_t CFGR;					/*	 Offset: 0x2c				Description: 	*/

}SYSCFG_RegDef_t;


#define SYSCFG ((SYSCFG_RegDef_t*) SYSFG_BASEADDR)




/*********************** RCC Register Definition Structures ***********************/


typedef struct{

	volatile uint32_t CR ;				/*	 Offset: 0x00	Description: 	*/
	volatile uint32_t PLLCFGR;			/*	 Offset: 0x04	Description: 	*/
	volatile uint32_t CFGR;				/*	 Offset: 0x08	Description: 	*/
	volatile uint32_t CIR;				/*	 Offset: 0x0C	Description: 	*/
	volatile uint32_t AHB1RSTR;			/*	 Offset: 0x10	Description: 	*/
	volatile uint32_t AHB2RSTR;			/*	 Offset: 0x14	Description: 	*/
	volatile uint32_t AHB3RSTR;			/*	 Offset: 0x18	Description: 	*/
	volatile uint32_t RESERVED0;   		/*	 Offset: 0x1C	Description: 	*/
	volatile uint32_t APB1RSTR;			/*	 Offset: 0x20	Description: 	*/
	volatile uint32_t APB2RSTR;			/*	 Offset: 0x24	Description: 	*/
	volatile uint32_t RESERVED1[2];   	/*	 Offset: 0x28	Description: 	*/
	volatile uint32_t AHB1ENR;			/*	 Offset: 0x30	Description: 	*/
	volatile uint32_t AHB2ENR;			/*	 Offset: 0x34	Description: 	*/
	volatile uint32_t AHB3ENR;			/*	 Offset: 0x38	Description: 	*/
	volatile uint32_t RESERVED2;   		/*	 Offset: 0x3C	Description: 	*/
	volatile uint32_t APB1ENR;			/*	 Offset: 0x40	Description: 	*/
	volatile uint32_t APB2ENR;			/*	 Offset: 0x44	Description: 	*/
	volatile uint32_t RESERVED3[2];   	/*	 Offset: 0x48	Description: 	*/
	volatile uint32_t AHB1LPENR;		/*	 Offset: 0x50	Description: 	*/
	volatile uint32_t AHB2LPENR;		/*	 Offset: 0x54	Description: 	*/
	volatile uint32_t AHB3LPENR;		/*	 Offset: 0x58	Description: 	*/
	volatile uint32_t RESERVED4;   		/*	 Offset: 0x5C	Description: 	*/
	volatile uint32_t APB1LPENR;		/*	 Offset: 0x60	Description: 	*/
	volatile uint32_t APB2LPENR;		/*	 Offset: 0x64	Description: 	*/
	volatile uint32_t RESERVED5[2];   	/*	 Offset: 0x68	Description: 	*/
	volatile uint32_t BDCR;				/*	 Offset: 0x70	Description: 	*/
	volatile uint32_t CSR;				/*	 Offset: 0x74	Description: 	*/
	volatile uint32_t RESERVED6[2];   	/*	 Offset: 0x78	Description: 	*/
	volatile uint32_t SSCGR;			/*	 Offset: 0x80	Description: 	*/
	volatile uint32_t PLLI2SCFGR;		/*	 Offset: 0x84	Description: 	*/
	volatile uint32_t PLLSAICFGR;		/*	 Offset: 0x88	Description: 	*/
	volatile uint32_t DCKCFGR;			/*	 Offset: 0x8C	Description: 	*/
	volatile uint32_t CKGATENR;			/*	 Offset: 0x90	Description: 	*/
	volatile uint32_t DCKCFGR2;			/*	 Offset: 0x94	Description: 	*/

}RCC_RegDef_t;


#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC ((RCC_RegDef_t*) RCC_BASEADDR)


#define GPIOA_BASEADDR_TO_CODE(x)	( 	(x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
										(x == GPIOE)?4:\
										(x == GPIOF)?5:\
										(x == GPIOG)?6:\
										(x == GPIOH)?7:0 )

//IRQ	(Interrupt Request) Numbers of STM32F446RE MCU

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40


//IRQ	Priority  Numbers of STM32F446RE MCU

#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15


/*********************** GPIO Enable/Disable Clock ***********************/


#define GPIOA_PCLCK_ENABLE()			( RCC->AHB1ENR |=  (1 << 0) )
#define GPIOA_PCLCK_DISABLE()			( RCC->AHB1ENR &= ~(1 << 0) )

#define GPIOB_PCLCK_ENABLE()			( RCC->AHB1ENR |=  (1 << 1) )
#define GPIOB_PCLCK_DISABLE()			( RCC->AHB1ENR &= ~(1 << 1) )

#define GPIOC_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 2) )
#define GPIOC_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 2) )

#define GPIOD_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 3) )
#define GPIOD_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 3) )

#define GPIOE_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 4) )
#define GPIOE_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 4) )

#define GPIOF_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 5) )
#define GPIOF_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 5) )

#define GPIOG_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 6) )
#define GPIOG_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 6) )

#define GPIOH_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 7) )
#define GPIOH_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 7) )

/*********************** GPIO rPIOx peripherals ***********************/

#define GPIOA_REG_RESET() 			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() 			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() 			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() 			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET() 			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET() 			do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET() 			do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET() 			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)


/*********************** GPIO low Power Enable/Disable Clock ***********************/


#define GPIOA_LP_PCLCK_ENABLE()			( RCC->AHB1LPENR |=  (1 << 0) )
#define GPIOA_LP_PCLCK_DISABLE()		( RCC->AHB1LPENR &= ~(1 << 0) )

#define GPIOB_LP_PCLCK_ENABLE()			( RCC->AHB1LPENR |=  (1 << 1) )
#define GPIOB_LP_PCLCK_DISABLE()		( RCC->AHB1LPENR &= ~(1 << 1) )

#define GPIOC_LP_PCLCK_ENABLE()			( RCC->AHB1LPENR |=   (1 << 2) )
#define GPIOC_LP_PCLCK_DISABLE()		( RCC->AHB1LPENR &=  ~(1 << 2) )

#define GPIOD_LP_PCLCK_ENABLE()			( RCC->AHB1LPENR |=   (1 << 3) )
#define GPIOD_LP_PCLCK_DISABLE()		( RCC->AHB1LPENR &=  ~(1 << 3) )

#define GPIOE_LP_PCLCK_ENABLE()			( RCC->AHB1LPENR |=   (1 << 4) )
#define GPIOE_LP_PCLCK_DISABLE()		( RCC->AHB1LPENR &=  ~(1 << 4) )

#define GPIOF_LP_PCLCK_ENABLE()			( RCC->AHB1LPENR |=   (1 << 5) )
#define GPIOF_LP_PCLCK_DISABLE()		( RCC->AHB1LPENR &=  ~(1 << 5) )

#define GPIOG_LP_PCLCK_ENABLE()			( RCC->AHB1LPENR |=   (1 << 6) )
#define GPIOG_LP_PCLCK_DISABLE()		( RCC->AHB1LPENR &=  ~(1 << 6) )

#define GPIOH_LP_PCLCK_ENABLE()			( RCC->AHB1LPENR |=   (1 << 7) )
#define GPIOH_LP_PCLCK_DISABLE()		( RCC->AHB1LPENR &=  ~(1 << 7) )


/*********************** CRC Enable/Disable Clock ***********************/

#define CRC_PCLCK_ENABLE()				( RCC->AHB1ENR |=   (1 << 12) )
#define CRC_PCLCK_DISABLE()				( RCC->AHB1ENR &=  ~(1 << 12) )

/*********************** CRC Low Power Enable/Disable Clock *************/

#define CRC_LP_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 12) )
#define CRC_LP_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 12) )

/*********************** FLITF Low Power Enable/Disable Clock *************/

#define FLITF_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 15) )
#define FLITF_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 15) )

/*********************** SRAM1 Low Power Enable/Disable Clock *************/

#define SRAM1_LP_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 16) )
#define SRAM1_LP_PCLCK_DISABLE()		( RCC->AHB1ENR &=  ~(1 << 16) )

/*********************** SRAM2 Low Power Enable/Disable Clock *************/

#define SRAM2_LP_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 17) )
#define SRAM2_LP_PCLCK_DISABLE()		( RCC->AHB1ENR &=  ~(1 << 17) )


/*********************** BKP SRAM Low Power Enable/Disable Clock *************/

#define BKPSRAM_LP_PCLCK_ENABLE()		( RCC->AHB1ENR |=   (1 << 18) )
#define BKPSRAM_LP_PCLCK_DISABLE()		( RCC->AHB1ENR &=  ~(1 << 18) )

/*********************** DMA1 Low Power Enable/Disable Clock *************/

#define DMA1_LP_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 21) )
#define DMA1_LP_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 21) )

/*********************** DMA2 Low Power Enable/Disable Clock *************/

#define DMA2_LP_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 22) )
#define DMA2_LP_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 22) )

/*********************** OTGHS Low Power Enable/Disable Clock *************/

#define OTGHS_LP_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 29) )
#define OTGHS_LP_PCLCK_DISABLE()		( RCC->AHB1ENR &=  ~(1 << 29) )

/*********************** OTGHSULPI Low Power Enable/Disable Clock *************/

#define OTGHSULPI_LP_PCLCK_ENABLE()		( RCC->AHB1ENR |=   (1 << 30) )
#define OTGHSULPI_LP_PCLCK_DISABLE()	( RCC->AHB1ENR &=  ~(1 << 30) )

/*********************** DMA1 Enable/Disable Clock ***********************/

#define DMA1_PCLCK_ENABLE()				( RCC->AHB1ENR |=   (1 << 21) )
#define DMA1_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 21) )

/*********************** DMA2 Enable/Disable Clock ***********************/

#define DMA2_PCLCK_ENABLE()				( RCC->AHB1ENR |=   (1 << 22) )
#define DMA2_PCLCK_DISABLE()			( RCC->AHB1ENR &=  ~(1 << 22) )

/*********************** BKP SRAMEN Enable/Disable Clock ***********************/

#define BKP_PCLCK_ENABLE()				( RCC->AHB1ENR |=   (1 << 18) )
#define BKP_PCLCK_DISABLE()				( RCC->AHB1ENR &=  ~(1 << 18) )

/*********************** OTGHS ULPIEN Enable/Disable Clock ***********************/

#define OTGHS_ULPIEN_PCLCK_ENABLE()		( RCC->AHB1ENR |=   (1 << 30) )
#define OTGHS_ULPIEN_PCLCK_DISABLE()	( RCC->AHB1ENR &=  ~(1 << 30) )

/*********************** OTGHS EN Enable/Disable Clock ***********************/

#define OTGHS_EN_PCLCK_ENABLE()			( RCC->AHB1ENR |=   (1 << 29) )
#define OTGHS_EN_PCLCK_DISABLE()		( RCC->AHB1ENR &=  ~(1 << 29) )


/*********************** TIM2 Enable/Disable Clock ***********************/

#define TIM2_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 0) )
#define TIM2_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 0) )

/*********************** TIM3 Enable/Disable Clock ***********************/

#define TIM3_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 1) )
#define TIM3_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 1) )

/*********************** TIM4 Enable/Disable Clock ***********************/

#define TIM4_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 2) )
#define TIM4_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 2) )

/*********************** TIM5 Enable/Disable Clock ***********************/

#define TIM5_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 3) )
#define TIM5_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 3) )

/*********************** TIM6 Enable/Disable Clock ***********************/

#define TIM6_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 4) )
#define TIM6_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 4) )

/*********************** TIM7 Enable/Disable Clock ***********************/

#define TIM7_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 5) )
#define TIM7_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 5) )

/*********************** TIM12 Enable/Disable Clock ***********************/

#define TIM12_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 6) )
#define TIM12_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 6) )

/*********************** TIM13 Enable/Disable Clock ***********************/

#define TIM13_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 7) )
#define TIM13_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 7) )

/*********************** TIM14 Enable/Disable Clock ***********************/

#define TIM14_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 8) )
#define TIM14_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 8) )

/*********************** WWDG Enable/Disable Clock ***********************/

#define WWDG_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 11) )
#define WWDG_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 11) )

/*********************** SPI2 Enable/Disable Clock ***********************/

#define SPI2_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 14) )
#define SPI2_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 14) )

/*********************** SPI3 Enable/Disable Clock ***********************/

#define SPI3_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 15) )
#define SPI3_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 15) )

/*********************** SPDIFRX Enable/Disable Clock ***********************/

#define SPDIFRX_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 16) )
#define SPDIFRX_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 16) )

/*********************** USART2 Enable/Disable Clock ***********************/

#define USART2_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 17) )
#define USART2_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 17) )

/*********************** USART3 Enable/Disable Clock ***********************/

#define USART3_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 18) )
#define USART3_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 18) )

/*********************** USART4 Enable/Disable Clock ***********************/

#define USART4_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 19) )
#define USART4_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 19) )

/*********************** USART5 Enable/Disable Clock ***********************/

#define USART5_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 20) )
#define USART5_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 20) )

/*********************** I2C1 Enable/Disable Clock ***********************/

#define I2C1_PCLCK_ENABLE()					( RCC->APB1ENR |=   (1 << 21) )
#define I2C1_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 21) )

/*********************** I2C2 Enable/Disable Clock ***********************/

#define I2C2_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 22) )
#define I2C2_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 22) )

/*********************** I2C3 Enable/Disable Clock ***********************/

#define I2C3_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 23) )
#define I2C3_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 23) )

/*********************** FMPI2C1 Enable/Disable Clock ***********************/

#define FMPI2C1_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 24) )
#define FMPI2C1_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 24) )

/*********************** CAN1 Enable/Disable Clock ***********************/

#define CAN1_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 25) )
#define CAN1_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 25) )

/*********************** CAN2 Enable/Disable Clock ***********************/

#define CAN2_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 26) )
#define CAN2_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 26) )

/*********************** CEC Enable/Disable Clock ***********************/

#define CEC_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 27) )
#define CEC_PCLCK_DISABLE()				( RCC->APB1ENR &=  ~(1 << 27) )

/*********************** PWR Enable/Disable Clock ***********************/

#define PWR_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 28) )
#define PWR_PCLCK_DISABLE()				( RCC->APB1ENR &=  ~(1 << 28) )

/*********************** DAC Enable/Disable Clock ***********************/

#define DAC_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 29) )
#define DAC_PCLCK_DISABLE()				( RCC->APB1ENR &=  ~(1 << 29) )


/*********************** WWDG Enable/Disable Clock ***********************/

#define WWDG_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 11) )
#define WWDG_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 11) )

/*********************** SPI2 Enable/Disable Clock ***********************/

#define SPI2_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 14) )
#define SPI2_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 14) )

/*********************** SPI3 Enable/Disable Clock ***********************/

#define SPI3_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 15) )
#define SPI3_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 15) )

/*********************** SPDIFRX Enable/Disable Clock ***********************/

#define SPDIFRX_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 16) )
#define SPDIFRX_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 16) )

/*********************** USART2 Enable/Disable Clock ***********************/

#define USART2_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 17) )
#define USART2_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 17) )

/*********************** USART3 Enable/Disable Clock ***********************/

#define USART3_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 18) )
#define USART3_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 18) )

/*********************** USART4 Enable/Disable Clock ***********************/

#define USART4_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 19) )
#define USART4_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 19) )

/*********************** USART5 Enable/Disable Clock ***********************/

#define USART5_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 20) )
#define USART5_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 20) )

/*********************** I2C1 Enable/Disable Clock ***********************/

#define I2C1_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 21) )
#define I2C1_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 21) )

/*********************** I2C2 Enable/Disable Clock ***********************/

#define I2C2_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 22) )
#define I2C2_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 22) )

/*********************** I2C3 Enable/Disable Clock ***********************/

#define I2C3_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 23) )
#define I2C3_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 23) )

/*********************** FMPI2C1 Enable/Disable Clock ***********************/

#define FMPI2C1_PCLCK_ENABLE()			( RCC->APB1ENR |=   (1 << 24) )
#define FMPI2C1_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 24) )

/*********************** CAN1 Enable/Disable Clock ***********************/

#define CAN1_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 25) )
#define CAN1_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 25) )

/*********************** CAN2 Enable/Disable Clock ***********************/

#define CAN2_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 26) )
#define CAN2_PCLCK_DISABLE()			( RCC->APB1ENR &=  ~(1 << 26) )

/*********************** CEC Enable/Disable Clock ***********************/

#define CEC_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 27) )
#define CEC_PCLCK_DISABLE()				( RCC->APB1ENR &=  ~(1 << 27) )

/*********************** PWR Enable/Disable Clock ***********************/

#define PWR_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 28) )
#define PWR_PCLCK_DISABLE()				( RCC->APB1ENR &=  ~(1 << 28) )

/*********************** DAC Enable/Disable Clock ***********************/

#define DAC_PCLCK_ENABLE()				( RCC->APB1ENR |=   (1 << 29) )
#define DAC_PCLCK_DISABLE()				( RCC->APB1ENR &=  ~(1 << 29) )

/*********************** TIM1 Enable/Disable Clock ***********************/

#define TIM1_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 0) )
#define TIM1_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 0) )

/*********************** TIM8 Enable/Disable Clock ***********************/

#define TIM8_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 1) )
#define TIM8_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 1) )

/*********************** USART1 Enable/Disable Clock ***********************/

#define USART1_PCLCK_ENABLE()			( RCC->APB2ENR |=   (1 << 4) )
#define USART1_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 4) )

/*********************** USART6 Enable/Disable Clock ***********************/

#define USART6_PCLCK_ENABLE()			( RCC->APB2ENR |=   (1 << 5) )
#define USART6_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 5) )

/*********************** ADC1 Enable/Disable Clock ***********************/

#define ADC1_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 8) )
#define ADC1_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 8) )

/*********************** ADC2 Enable/Disable Clock ***********************/

#define ADC2_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 9) )
#define ADC2_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 9) )

/*********************** ADC3 Enable/Disable Clock ***********************/

#define ADC3_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 10) )
#define ADC3_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 10) )

/*********************** SDIO Enable/Disable Clock ***********************/

#define SDIO_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 11) )
#define SDIO_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 11) )

/*********************** SPI1 Enable/Disable Clock ***********************/

#define SPI1_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 12) )
#define SPI1_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 12) )

/*********************** SPI4 Enable/Disable Clock ***********************/

#define SPI4_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 13) )
#define SPI4_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 13) )

/*********************** SYSCFG Enable/Disable Clock ***********************/

#define SYSCFG_PCLCK_ENABLE()			( RCC->APB2ENR |=   (1 << 14) )
#define SYSCFG_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 14) )

/*********************** TIM9 Enable/Disable Clock ***********************/

#define TIM9_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 16) )
#define TIM9_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 16) )

/*********************** TIM10 Enable/Disable Clock ***********************/

#define TIM10_PCLCK_ENABLE()			( RCC->APB2ENR |=   (1 << 17) )
#define TIM10_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 17) )

/*********************** TIM11 Enable/Disable Clock ***********************/

#define TIM11_PCLCK_ENABLE()			( RCC->APB2ENR |=   (1 << 18) )
#define TIM11_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 18) )

/*********************** SAI1 Enable/Disable Clock ***********************/

#define SAI1_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 22) )
#define SAI1_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 22) )

/*********************** SAI2 Enable/Disable Clock ***********************/

#define SAI2_PCLCK_ENABLE()				( RCC->APB2ENR |=   (1 << 23) )
#define SAI2_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 23) )

/*********************** DCMI Enable/Disable Clock ***********************/

#define DCMI_PCLCK_ENABLE()				( RCC->AHB2LPENR |=   (1 << 0) )
#define DCMI_PCLCK_DISABLE()			( RCC->AHB2LPENR &=  ~(1 << 0) )

/*********************** OTGFS Enable/Disable Clock ***********************/

#define OTGFS_PCLCK_ENABLE()			( RCC->APB2ENR |=   (1 << 7) )
#define OTGFS_PCLCK_DISABLE()			( RCC->APB2ENR &=  ~(1 << 7) )

/*********************** FMC Enable/Disable Clock ***********************/

#define FMC_PCLCK_ENABLE()				( RCC->AHB3LPENR |=   (1 << 0) )
#define FMC_PCLCK_DISABLE()				( RCC->AHB3LPENR &=  ~(1 << 0) )

/*********************** QSPI Enable/Disable Clock ***********************/

#define QSPCI_PCLCK_ENABLE()			( RCC->AHB3LPENR |=   (1 << 1) )
#define QSPCI_PCLCK_DISABLE()			( RCC->AHB3LPENR &=  ~(1 << 1) )



/*	HARDWARE OF NUCLEO BOARD NUCLEO 32F446RE*/

#define GPIO_USER_LED		GPIOA
#define LED_USER_NUCLEO		5

#define GPIO_USER_BUTTON	GPIOC
#define BUTTON_USER_NUCLEO 	13

#endif /* INC_STM32F446XX_H_ */
