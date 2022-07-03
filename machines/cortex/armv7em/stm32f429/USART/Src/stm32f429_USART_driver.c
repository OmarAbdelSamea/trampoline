#include "../inc/stm32f429_usart_driver.h"

/*
 * =======================================================================================
 * 							Generic Variables
 * =======================================================================================
 */
uart_Config* Global_uart_Config  = NULL ;

/*
 * =======================================================================================
 * 							Generic Functions
 * =======================================================================================
 */

/**================================================================
 * @Fn				-MCAL_uart_Init
 * @brief 			- Initializes uart (Supported feature ASYNCH. Only)
 * @param [in] 		- usartx: where x can be (1..3 depending on device used)
 * @param [in] 		- uart_Config: All uart Configuration EXTI_PinConfig_t
 * @retval 			-none
 * Note				-Support for Now Asynch mode & Clock 8 MHZ S
 */
void MCAL_uart_Init (usart_TypeDef *usartx, uart_Config* uart_Config)
{
	Global_uart_Config = uart_Config ;
	uint32_t pclk ,BRR  ;
	//	enable the Clock for given usart peripheral

	if ( usartx == usart1 )
		rcc_usart1_CLK_EN();

	else if ( usartx == usart2 )
		rcc_usart2_CLK_EN();

	else if ( usartx == usart3 )
		rcc_usart3_CLK_EN();

	else if ( usartx == usart6 )
		rcc_usart6_CLK_EN();
	//Enable usart Module
	//	usart_CR1  Bit 13 UE: usart enable
	usartx->CR1 |= 1<<13 ;

	//Enable usart Tx and Rx engines according to the usart_Mode configuration item
	//	usart_CR1 Bit 3 TE: Transmitter enable & Bit 2 RE: Receiver enable
	usartx->CR1 |= uart_Config->usart_Mode ;

	//PAYLOAD Width
	// usartx->CR1  Bit 12 M: Word length
	usartx->CR1 |= uart_Config->Payload_Length ;

	//Configuration of parity control bit fields
	// usartx->CR1 	Bit 10 PCE: Parity control enable     Bit 9 PS: Parity selection
	usartx->CR1 |= uart_Config->Parity ;

	//configure the number of stop bits
	//usart_CR2  Bits 13:12 STOP: STOP bits
	usartx->CR2 |= uart_Config->StopBits ;


	//usart hardware flow control
	//usart_CR3  Bit 9 CTSE: CTS enable   Bit 8 RTSE: RTS enable
	usartx->CR3 |= uart_Config->HwFlowCtl ;


	//Configuration of BRR(Baudrate register)
	//pclk1 = HCLK / APB1Prescaler = 180MHz / 8 = 22.5 MHz for APB1 Peripherals (usart2, usart3)
	//pclk2 = HCLK / APB1Prescaler = 180MHz / 2 = 90 MHz for APB1 Peripherals (usart1, usart6)
	pclk = 16000000;
	if ( usartx == usart2 || usartx == usart3)
	{
//		pclk = 22500000;
	}
	else if (usartx == usart1 || usartx == usart6 )
	{
//		pclk = 90000000;
	}

	BRR = uart_BRR_Register(pclk, uart_Config->BaudRate) ;
	usartx->BRR = BRR ;
	//ENABLE / DISABLE Interrupt
	//usart_CR1
	if (uart_Config->IRQ_Enable  != uart_IRQ_Enable_NONE)
	{
		usartx->CR1 |= (uart_Config->IRQ_Enable) ;
		//		Enable nvic For usartx IRQ
		if ( usartx == usart1 )
			nvic_IRQ37_usart1_Enable ;

		else if ( usartx == usart2 )
			nvic_IRQ38_usart2_Enable ;

		else if ( usartx == usart3 )
			nvic_IRQ39_usart3_Enable ;
	}
}

/**================================================================
 * @Fn				-MCAL_uart_DEInit
 * @brief 			- DEInitializes uart (Supported feature ASYNCH. Only)
 * @param [in] 		- usartx: where x can be (1..3 depending on device used)
 * @retval 			-none
 * Note				-Reset the Model By rcc
 */
void MCAL_uart_DeInit (usart_TypeDef *usartx)
{
	if ( usartx == usart1 )
	{
		rcc_usart1_Reset();
		nvic_IRQ37_usart1_Disable ;

	}
	else if ( usartx == usart2 )
	{
		rcc_usart2_Reset();
		nvic_IRQ38_usart2_Disable ;
	}
	else if ( usartx == usart3 )
	{
		rcc_usart3_Reset();
		nvic_IRQ39_usart3_Disable ;
	}
}

/*********************************************************************
 * @fn      		  - MCAL_uart_SendData
 *
 * @brief             -Send Buffer on uart
 *
 * @param [in] 		- usartx: where x can be (1..3 depending on device used)
 * @param[in]         -pTxBuffer Buffer
 * @param[in]         -PollingEn   Enable pooling or dsable it
 *
 * @return            -

 * @Note              - Should initialize uart First
 * 		//			When transmitting with the parity enabled (PCE bit set to 1 in the usart_CR1 register),
			//			the value written in the MSB (bit 7 or bit 8 depending on the data length) has no effect
			//			because it is replaced by the parity.
			//			When receiving with the parity enabled, the value read in the MSB bit is the received parity
			//			bit

 */

void MCAL_uart_SendData	(usart_TypeDef *usartx, uint16_t *pTxBuffer,enum Polling_mechism PollingEn )
{
	// wait until TXE flag is set in the SR
	if (PollingEn == enable)
		while(! (usartx->SR & 1<<7 ) );

	//Check the usart_WordLength item for 9BIT or 8BIT in a frame
	if (Global_uart_Config->Payload_Length == uart_Payload_Length_9B)
	{
		//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits

		//			When transmitting with the parity enabled (PCE bit set to 1 in the usart_CR1 register),
		//			the value written in the MSB (bit 7 or bit 8 depending on the data length) has no effect
		//			because it is replaced by the parity.
		//			When receiving with the parity enabled, the value read in the MSB bit is the received parity
		//			bit.
		//
		usartx->DR = (*pTxBuffer & (uint16_t)0x01FF);
	}else
	{
		//This is 8bit data transfer
		usartx->DR = (*pTxBuffer  & (uint8_t)0xFF);
	}
}

void MCAL_uart_WAIT_TC (usart_TypeDef *usartx )
{
	// wait till TC flag is set in the SR
	while( ! (usartx->SR & 1<<6 ));
}

void MCAL_uart_ReceiveData	(usart_TypeDef *usartx, uint16_t *pRxBuffer ,enum Polling_mechism PollingEn )
{
	//Loop over until "Len" number of bytes are transferred
	//wait until RXNE flag is set in the SR
	if (PollingEn == enable)
	{
		while( ! (usartx->SR & 1<<5 ));
	}

	//Check the usart_WordLength item for 9BIT or 8BIT in a frame
	if (Global_uart_Config->Payload_Length == uart_Payload_Length_9B)
	{
		if (Global_uart_Config->Parity ==uart_Parity__NONE)
		{
			//no parity So all 9bit are considered data
			*((uint16_t*) pRxBuffer) = usartx->DR ;
		}else
		{
			//Parity is used, so, 8bits will be of user data and 1 bit is parity
			*((uint16_t*) pRxBuffer) = ( usartx->DR  & (uint8_t)0xFF );
		}
	}else
	{
		//This is 8bit data
		if (Global_uart_Config->Parity ==uart_Parity__NONE)
		{
			//no parity So all 8bit are considered data
			*((uint16_t*) pRxBuffer) = ( usartx->DR  & (uint8_t)0xFF ) ;
		}else
		{
			//Parity is used, so,7 bits will be of user data and 1 bit is parity
			*((uint16_t*) pRxBuffer) = ( usartx->DR  & (uint8_t)0X7F );
		}
	}
}


void MCAL_uart_gpio_Set_Pins (usart_TypeDef *usartx)
{
	if(usartx == usart1)
	{
		volatile uint32_t delay = 0;
		/*PA10 & PA9 Tx&Rx*/
		/*enable to gpioA clock*/
		rcc_gpioA_CLK_EN();
		delay = 5;
		/* enable alternative function  to pin 9*/
		gpioA->MODER |= (2<<18);
		/* enable alternative function  to pin 10*/
		gpioA->MODER |= (2<<20);

		/*configure pins to very high speed*/
		gpioA->OSPEEDR |= (3<<18);
		gpioA->OSPEEDR |= (3<<20);

		/*AF7 Alternative function for uart1 for pins PA10 &PA9*/
		gpioA->AFRH |= (7<<4);
		gpioA->AFRH |= (7<<8);
		delay++;
	}
	else if (usartx == usart2)
	{
		volatile uint32_t delay = 0;
		/*PA2 & PA3 Tx&Rx*/
		/*enable to gpioA clock*/
		rcc_gpioA_CLK_EN();
		delay = 5;
		/* enable alternative function  to pin 3 RX*/
		gpioA->MODER |= (2<<6);
		/* enable alternative function  to pin 2 TX*/
		gpioA->MODER |= (2<<4);

		/*configure pins to very high speed*/
		gpioA->OSPEEDR |= (3<<6);
		gpioA->OSPEEDR |= (3<<4);

		/*AF7 Alternative function for uart2 for pins PA3 &PA2*/
		gpioA->AFRL |= (7<<8);
		gpioA->AFRL |= (7<<12);
		delay++;
	}
	else if (usartx == usart3)
	{
		volatile uint32_t delay = 0;
		/*PB10 & PB11 Tx&Rx*/
		/*enable to gpioB clock*/
		rcc_gpioB_CLK_EN();
		delay = 5;
		/* enable alternative function  to pin 11 RX*/
		gpioB->MODER |= (2<<22);
		/* enable alternative function  to pin 10 TX*/
		gpioB->MODER |= (2<<20);

		/*configure pins to very high speed*/
		gpioB->OSPEEDR |= (3<<22);
		gpioB->OSPEEDR |= (3<<20);

		/*AF7 Alternative function for uart3 for pins PB10 &PB11*/
		gpioB->AFRH |= (7<<8);
		gpioB->AFRH |= (7<<12);
		delay++;
	}
	else if (usartx == usart6)
	{
		volatile uint32_t delay = 0;
		/*PC6 & PC7 Tx&Rx*/
		/*enable to gpioC clock*/
		rcc_gpioC_CLK_EN();
		delay = 5;
		/* enable alternative function  to pin 7 RX*/
		gpioC->MODER |= (2<<14);
		/* enable alternative function  to pin 6 TX*/
		gpioC->MODER |= (2<<12);

		/*configure pins to very high speed*/
		gpioC->OSPEEDR |= (3<<14);
		gpioC->OSPEEDR |= (3<<12);

		/*AF7 Alternative function for uart6 for pins PC6 &PC7*/
		gpioC->AFRL |= (8<<24);
		gpioC->AFRL |= (8<<28);
		delay++;
	}

}

//ISR
void usart1_IRQHandler (void)
{
	Global_uart_Config->P_IRQ_CallBack () ;
}

void usart2_IRQHandler (void)
{
	Global_uart_Config->P_IRQ_CallBack () ;
}
void usart3_IRQHandler (void)
{
	Global_uart_Config->P_IRQ_CallBack () ;
}
