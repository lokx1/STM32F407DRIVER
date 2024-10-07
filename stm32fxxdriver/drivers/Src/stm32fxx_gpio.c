/*
 * stm32fxx_gpio.c
 *
 *  Created on: Jul 29, 2024
 *      Author: ADMIN
 */


#include "stm32fxx_gpio.h"
/*
 * Peripheral Clock setup
 */
int count =5;
void GPIO_PeriClockControl(GPIO_RegDef_t *GPIO,uint8_t state){


	if(state==ENABLE)
	{
		if(GPIO==GPIOA){
					GPIOA_PERI_CLOCK_EN();
		} else if(GPIO==GPIOB){
					GPIOB_PERI_CLOCK_EN();
		} else if(GPIO==GPIOC){
					GPIOC_PERI_CLOCK_EN();
		} else if(GPIO==GPIOD){
					GPIOD_PERI_CLOCK_EN();
		}else if(GPIO==GPIOE){
					GPIOE_PERI_CLOCK_EN();
		}
		else if(GPIO==GPIOF){
					GPIOF_PERI_CLOCK_EN();
				}
		else if(GPIO==GPIOG){
					GPIOG_PERI_CLOCK_EN();
				}
		else if(GPIO==GPIOH){
					GPIOH_PERI_CLOCK_EN();
				}
		else if(GPIO==GPIOI){
					GPIOI_PERI_CLOCK_EN();
						}


	}else {



	}



}

/*
 * Init and De-Init
 */
void GPIO_Init(GPIOx_Handle_t *pGPIOHandle){

	uint32_t temp=0;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//1. Configure the mode of the pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){

		//none interupt mode
		temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &=~(0x3<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
		// each bit fields take 2
		pGPIOHandle->pGPIOx->MODER |= temp;

		//DBA

	}else {
		//interupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){

			//1. Configure Falling Trigger Selection Reg
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clearing corresponding RTSR
			EXTI->RTSR &=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){

			//1. Configure the RTSR and FTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT){


			//1.Configure the RFTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clearing corresponding RTSR
			EXTI->RTSR &=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2.Configure the port selection in SYSCFG_EXTI
		uint8_t temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint8_t temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4  ;
		uint8_t portcode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]= portcode <<(temp2*4);

		//3. Enable EXTI interupt delivery using IMR

		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}
	temp=0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}


}
void GPIO_DeInit(GPIO_RegDef_t *GPIO){
					if(GPIO==GPIOA){
								GPIOA_REG_RESET();
					} else if(GPIO==GPIOB){
						GPIOB_REG_RESET();
					} else if(GPIO==GPIOC){
						GPIOC_REG_RESET();
					} else if(GPIO==GPIOD){
						GPIOD_REG_RESET();
					}else if(GPIO==GPIOE){
						GPIOE_REG_RESET();
					}
					else if(GPIO==GPIOF){
						GPIOF_REG_RESET();
							}
					else if(GPIO==GPIOG){
						GPIOG_REG_RESET();
							}
					else if(GPIO==GPIOH){
						GPIOH_REG_RESET();
							}
					else if(GPIO==GPIOI){
						GPIOI_REG_RESET();
									}




}
/*
 * Data read and write
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){

	  uint8_t value;

	   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 ) ;

	   return value;

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value =(uint16_t)pGPIOx->IDR;

	return value;



}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value){

	if(value==GPIO_PIN_SET){
		pGPIOx->ODR |=(1<< PinNumber);


	}else {

		pGPIOx->ODR &=~(1<< PinNumber);

	}


}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value){
	pGPIOx->ODR=value;


}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR  ^= ( 1 << PinNumber);


}
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t state){

if(state==ENABLE){

	if(IRQNumber <=31){

		//program ISER0 reg
		*NVIC_ISER0 |= (1<<IRQNumber);


	}else if(IRQNumber >31 && IRQNumber <64){


		//program ISER1 reg
		*NVIC_ISER1 |= (1<<IRQNumber %32);

	}else if(IRQNumber >=64 && IRQNumber <96){


		//program ISER2 reg
		*NVIC_ISER3 |= (1<<IRQNumber %64);


	}


}else {
	if(IRQNumber <=31){

			//program ICER0 reg
		*NVIC_ICER0 |= (1<<IRQNumber);


		}else if(IRQNumber >31 && IRQNumber <64){


			//program ICER1 reg
			*NVIC_ICER1 |= (1<<IRQNumber%32);

		}else if(IRQNumber >=64 && IRQNumber <96){


			//program ICER2 reg

			*NVIC_ICER3 |= (1<<IRQNumber%64);

		}



}





}
void GPIO_IRQHandling(uint8_t pin) {

	if(EXTI->PR &(1<<pin)){
		//clear
		EXTI->PR |= (1 << pin);



	}

}
	void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

		//1. first lets find out the ipr register
			uint8_t iprx = IRQNumber / 4;
			uint8_t iprx_section  = IRQNumber %4 ;

			uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

			*( NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );




	}








