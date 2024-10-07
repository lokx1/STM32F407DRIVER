/*
 * stm32fxx_spi.h
 *
 *  Created on: Aug 25, 2024
 *      Author: ADMIN
 */

#ifndef INC_STM32FXX_SPI_H_
#define INC_STM32FXX_SPI_H_

#include "stm32fxx.h"

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


typedef struct {

	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;


}SPI_Handle_t;


#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * Bus CFG
 */
#define SPI_BUS_CFG_FD				1
#define SPI_BUS_CFG_HD				2
#define SPI_BUS_CFG_S_RX			3 //simplex TX = FULL DUPLEX

/*
 * SPI_SclkSPeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7
/*
 * DFF
 */
#define SPI_DFF_8					0
#define SPI_DFF_16					1


/*
 * CPOL
 */
#define SPI_CPOL_HIGH 				1
#define SPI_CPOL_LOW				0
/*
 * CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW  0
/*
 * SSM
 */
#define SPI_SSM_EN		1
#define SPI_SSM_DI		0
/*
 * SPI related status flag definition
 */
#define SPI_TXE_FLAG			(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG			(1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG			(1<<SPI_SR_BSY)


/*
 * Peripheral Clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t*pSPIx,uint8_t state);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);






void SPI_DeInit(SPI_RegDef_t *pSPIx);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t flagName);
/*
 * Data send and Receive
 */


void SPI_Send(SPI_RegDef_t *pSPIx,uint8_t*pTxBuffer,uint32_t len);
void SPI_Receive(SPI_RegDef_t *pSPIx,uint8_t*pRxBuffer,uint32_t len);



/*
 * IRQ Handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t state);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
/*
 *Other Peripheral controls APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t state);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t state);

#endif /* INC_STM32FXX_SPI_H_ */
