/*
 * stm32fxx.h
 *
 *  Created on: Jul 29, 2024
 *      Author: ADMIN
 */

#ifndef INC_STM32FXX_H_
#define INC_STM32FXX_H_

#include<stdint.h>

#define __vo 								volatile


/*
 * NVIC ISERx reg addr
 */
#define NVIC_ISER0		((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10C)



/*
 * NVIC ICERx reg
 */

#define NVIC_ICER0		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1		((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2		((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3		((__vo uint32_t*)0xE000E18C)

#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)





/*BASE ADDRESS OF FLASH AND RAM ADDRESS*/
#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define SRAM2_BASEADDR						0x20001C00U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM 								SRAM1_BASEADDR
/*
 * AHBx and APBx BUS Peripheral base addresses
 *
 * */
#define PERIPH_BASE							0x40000000U
#define	APB1PERIPH_BASE						PERIPH_BASE
#define APB2PERIPH_BASE						0x40010000U
#define AHB1PERIPH_BASE						0x40020000U
#define AHB2PERIPH_BASE						0x50000000U
/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR						(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR						(AHB1PERIPH_BASE + 0x2000)

#define RCC_BASEADDR						(AHB1PERIPH_BASE + 0x3800)
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASE + 0x5C00)

#define	SPI2_BASEADDR						(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASE + 0x3C00)


#define USART2_BASEADDR						(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASE + 0x5000)
/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASE+0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASE+0x3000)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASE+0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASE+0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASE+0x1400)





/*
 *
 */
typedef struct{
	__vo	uint32_t MODER;
	__vo	uint32_t OTYPER;
	__vo	uint32_t OSPEEDR;
	__vo	uint32_t PUPDR;
	__vo	uint32_t IDR;
	__vo	uint32_t ODR;
	__vo	uint32_t BSRR;
	__vo	uint32_t LCKR;
	__vo	uint32_t AFR[2];


}GPIO_RegDef_t;




//RCC reg def



typedef struct {
	 __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
	  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
	  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
	  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
	  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
	  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
	  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
	  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
	  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
	  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
	  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
	  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
	  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
	  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
	  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
	  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
	  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
	  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
	  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
	  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
	  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
	  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
	  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
	  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
	  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
	  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
	  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
	  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
	  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
	  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
	  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
	  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
	  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
	  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

}RCC_RegDef_t;

typedef struct {

		  __vo uint32_t IMR;        	 /*!< TODO,     										Address offset: 0x00 */
		  __vo uint32_t EMR;    		 /*!< TODO,     										Address offset: 0x04 */
		  __vo uint32_t RTSR;    	     /*!< TODO,     										Address offset: 0x08 */
		  __vo uint32_t FTSR;      		 /*!< TODO,     										Address offset: 0x0C */
		  __vo uint32_t SWIER;     		 /*!< TODO,     										Address offset: 0x10 */
		  __vo uint32_t PR;    		     /*!< TODO,  											Address offset: 0x14 */


}EXTI_RegDef_t;


typedef struct {

		  __vo uint32_t MEMRMP;        	 /*!< TODO,     										Address offset: 0x00 */
		  __vo uint32_t PMC;    		 /*!< TODO,     										Address offset: 0x04 */
		  __vo uint32_t EXTICR[4];    	 /*!< TODO,     										Address offset: 0x08-0x14 */
		  uint32_t 		RESERVED1;       /*!< TODO,     						             	Reserved: 0x18-0x1C */
		  __vo uint32_t CMPCR;     		 /*!< TODO,												 Address offset: 0x20 */
		  uint32_t 		RESERVED2[2];
		  __vo uint32_t CEGR;    		     /*!< TODO,  											Address offset: 0x2C */





}SYSCFG_RegDef_t;

/*
 * SPI
 */
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;


}SPI_RegDef_t;



#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)


#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)


#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)



#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)






#define GPIO_BASEADDR_TO_CODE(x)	   ((x==GPIOA)?0:\
										(x==GPIOB)?1:\
										(x==GPIOC)?2:\
										(x==GPIOD)?3:\
										(x==GPIOE)?4:\
										(x==GPIOF)?5:\
										(x==GPIOG)?6:\
										(x==GPIOH)?7:\
										(x==GPIOI)?8:0		)
/*
 * CLOCK ENABLE
 */
#define GPIOA_PERI_CLOCK_EN()		(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PERI_CLOCK_EN()		(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PERI_CLOCK_EN()		(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PERI_CLOCK_EN()		(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PERI_CLOCK_EN()		(RCC->AHB1ENR |=(1<<4))
#define GPIOF_PERI_CLOCK_EN()		(RCC->AHB1ENR |=(1<<5))
#define GPIOG_PERI_CLOCK_EN()		(RCC->AHB1ENR |=(1<<6))
#define GPIOH_PERI_CLOCK_EN()		(RCC->AHB1ENR |=(1<<7))
#define GPIOI_PERI_CLOCK_EN()		(RCC->AHB1ENR |=(1<<8))

/*
 * CLOCK ENABLE FOR SPIx
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * CLOCK ENABLE FOR I2Cx
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))
/*
 * CLock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5)))

/*
 * Clock disable Macros for SPIx
 */


/*
 * Clock disable Macros for GPIOx
 */
#define GPIOA_REG_RESET()		do{(RCC->AHB1ENR |=(1<<0));(RCC->AHB1ENR &=~(1<<0));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1ENR |=(1<<1));(RCC->AHB1ENR &=~(1<<1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1ENR |=(1<<2));(RCC->AHB1ENR &=~(1<<2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1ENR |=(1<<3));(RCC->AHB1ENR &=~(1<<3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1ENR |=(1<<4));(RCC->AHB1ENR &=~(1<<4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1ENR |=(1<<5));(RCC->AHB1ENR &=~(1<<5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1ENR |=(1<<6));(RCC->AHB1ENR &=~(1<<6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1ENR |=(1<<7));(RCC->AHB1ENR &=~(1<<7));}while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1ENR |=(1<<8));(RCC->AHB1ENR &=~(1<<8));}while(0)
/*
 * Clock disable SPI
 */
#define SPI1_PCLK_DIS()			do{(RCC->APB2ENR |= (1 << 12));(RCC->APB2ENR &=~(1 << 12));}while(0)
#define SPI2_PCLK_DIS()			do{(RCC->APB1ENR |= (1 << 14));(RCC->APB1ENR &= ~(1 << 14));}while(0)
#define SPI3_PCLK_DIS()			do{(RCC->APB1ENR |= (1 << 15));(RCC->APB1ENR &= ~(1 << 15));}while(0)
/*
 * Clock Enable for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR|=(1<<14))


#define IRQ_NO_EXTI0 				6
#define IRQ_NO_EXTI1 				7
#define IRQ_NO_EXTI2 				8
#define IRQ_NO_EXTI3 				9
#define IRQ_NO_EXTI4 				10
#define IRQ_NO_EXTI9_5 				23
#define IRQ_NO_EXTI15_10 			40


#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15









/*
 * SOME GENERIC MACROS
 */
#define ENABLE						 1
#define DISABLE 					 0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_RESET					RESET
#define FLAG_SET					SET

/*
 *		Bit position macros for SPI peripheral CR1
 *
 *
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR  		3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_EN			13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15
/*
 * 	CR2
 *
 *
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR1_RXNEIE		6
#define SPI_CR2_TXEIE		7




/*
 * SR
 */

#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

#include "stm32fxx_gpio.h"
#include "stm32fxx_spi.h"
#endif /* INC_STM32FXX_H_ */
