/*
 * stm32fxx_spi.h
 *
 *  Created on: Aug 25, 2024
 *      Author: ADMIN
 */
#include "stm32fxx_spi.h"



/*
 * Peripheral Clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t*pSPIx,uint8_t state){
	if(state==ENABLE)
		{
			if(pSPIx==SPI1){
				SPI1_PCLK_EN();
			} else if(pSPIx==SPI2){
				SPI2_PCLK_EN();
			} else if(pSPIx==SPI3){
				SPI3_PCLK_EN();
			}
		}else {
		}

}

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//CLOCK ENABLE

		SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);


		uint32_t temp=0;

		//1. Configure the device mode

		temp|=pSPIHandle->SPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR;

		//2. Bus config
		if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CFG_FD){

			//bidi mode should be clear
			temp &=~(1<<SPI_CR1_BIDIMODE);

		}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CFG_HD){

			//bidi mode should be en
			temp |=(1<<SPI_CR1_BIDIMODE);

		}else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CFG_S_RX){

			//bidi mode should be clear
			temp &=~(1<<SPI_CR1_BIDIMODE);
			//RX only  must be set
			temp |=(1<<SPI_CR1_RXONLY);
		}
		//3. Configure the spi serial clock speed
		temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;
		//4. DFF
		temp|= pSPIHandle->SPIConfig.SPI_DFF<< SPI_CR1_DFF;
		//5. CPOL
		temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
		//6. CPHA
		temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
		//7.SSM
		temp |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
		pSPIHandle->pSPIx->CR1 = temp;







}
void SPI_DeInit(SPI_RegDef_t *pSPIx){


			if(pSPIx==SPI1){
					SPI1_PCLK_DIS();
				} else if(pSPIx==SPI2){
					SPI2_PCLK_DIS();
				} else if(pSPIx==SPI3){
					SPI3_PCLK_DIS();
				}








}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t flagName){

	if(pSPIx->SR & flagName){

		return FLAG_SET;

	}
	return FLAG_RESET;


}
/*
 * Data send and Receive
 */
// Blocking call -> phai truyen het data thi function moi tra ve

void SPI_Send(SPI_RegDef_t *pSPIx,uint8_t*pTxBuffer,uint32_t len){
	while(len>0){
		//Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET);

		// Check the DFF bit
		if(pSPIx->CR1 &(1<<SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. Load the date in to the register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		}else {
			//8 bit
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;

		}

	}





}
void SPI_Receive(SPI_RegDef_t *pSPIx,uint8_t*pRxBuffer,uint32_t len);



/*
 * IRQ Handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t state);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
/*
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t state){

	if(state==ENABLE){
		pSPIx->CR1|= (1<<SPI_CR1_SPE);
	}else{

		pSPIx->CR1&= ~(1<<SPI_CR1_SPE);
	}

}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t state){

	if(state==ENABLE){
		pSPIx->CR1|= (1<<SPI_CR1_SSI);
	}else{

		pSPIx->CR1&= ~(1<<SPI_CR1_SSI);
	}

}




