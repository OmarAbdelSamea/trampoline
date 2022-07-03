#ifndef STM32F429_usart_DRIVER_H_
#define STM32F429_usart_DRIVER_H_

//includes
#include "STM32F429.h"

//Configuration structure
typedef struct
{
	uint32_t        			  usart_Mode;     		  // Specifies the TX/RX Mode.
	// This parameter must be set based on @ref uart_Mode_define
	uint32_t  			      BaudRate ; 		    // This member configures the uart communication baud rate
	// This parameter must be set based on @ref uart_BaudRate_define
	uint32_t 				  Payload_Length;			// Specifies the number of data bits transmitted or received in a frame.
	// This parameter must be set based on @ref uart_Payload_Length_define
	uint32_t 				  Parity ;					//Specifies the parity mode.
	//@ref uart_Parity_define
	uint32_t 				  StopBits ;				//Specifies the number of stop bits transmitted
	//@ref uart_StopBits_define
	uint32_t 				  HwFlowCtl ;				//Specifies whether the hardware flow control mode is enabled or disabled
	//@ref uart_HwFlowCtl_define
	uint32_t					 IRQ_Enable;				//enable or disable uart IRQ TX/RX
	//@ref uart_IRQ_Enable_define , you can select two or three parameters EX.uart_IRQ_Enable_TXE |uart_IRQ_Enable_TC

	void(* P_IRQ_CallBack)(void) ;					//Set the C Function() which will be called once the IRQ  Happen

}uart_Config;

// * =======================================================================================
//Reference Macros
// * =======================================================================================

//uart_Mode_define

#define uart_MODE_RX                        (uint32_t) (1<<2)  //RE =1
#define uart_MODE_TX                       	(uint32_t) (1<<3) //TE =1
#define uart_MODE_TX_RX                     ((uint32_t)(1<<2 | 1<<3))

//uart_BaudRate_define

#define uart_BaudRate_2400                   2400
#define uart_BaudRate_9600                   9600
#define uart_BaudRate_19200                  19200
#define uart_BaudRate_57600                  57600
#define uart_BaudRate_115200                 115200
#define uart_BaudRate_230400                 230400
#define uart_BaudRate_460800                 460800
#define uart_BaudRate_921600                 921600
#define uart_BaudRate_2250000                2250000
#define uart_BaudRate_4500000                4500000


//uart_Payload_Length_define

#define uart_Payload_Length_8B                  (uint32_t)(0)
#define uart_Payload_Length_9B                  (uint32_t)(1<<12)

//@ref uart_Parity_define

#define uart_Parity__NONE                    (uint32_t)(0)
#define uart_Parity__EVEN                    ((uint32_t)1<<10)
#define uart_Parity__ODD                     ((uint32_t)(1<<10 | 1<<9))

//@ref uart_StopBits_define

#define uart_StopBits__half                  (uint32_t)(1<<12)
#define uart_StopBits__1                     (uint32_t)(0)
#define uart_StopBits__1_half                (uint32_t)(3<<12)
#define uart_StopBits__2                     (uint32_t)(2<<12)

//@ref uart_HwFlowCtl_define

#define uart_HwFlowCtl_NONE                  (uint32_t)(0)
#define uart_HwFlowCtl_RTS                   ((uint32_t)1<<8)
#define uart_HwFlowCtl_CTS                   ((uint32_t)1<<9)
#define uart_HwFlowCtl_RTS_CTS               ((uint32_t)(1<<8 | 1<<9))

//@ref uart_IRQ_Enable_define
#define uart_IRQ_Enable_NONE                      		(uint32_t)(0)
#define uart_IRQ_Enable_TXE                       		(uint32_t) (1<<7) //Transmit data register empty
#define uart_IRQ_Enable_TC                     			((uint32_t)(1<<6)) //Transmission complete
#define uart_IRQ_Enable_RXNEIE                       	(uint32_t) (1<<5) //Received data ready to be read & Overrun error detected
#define uart_IRQ_Enable_PE                       	     (uint32_t) (1<<8) //Parity error

//BaudRate Calculation
//usartDIV = fclk / (16 * Baudrate)
//usartDIV_MUL100 =
// uint32((100 *fclk ) / (16 * Baudrate) == (25 *fclk ) / (4* Baudrate) )
//DIV_Mantissa_MUL100 = Integer Part (usartDIV  )  * 100
//DIV_Mantissa = Integer Part (usartDIV  )
//DIV_Fraction = (( usartDIV_MUL100  - DIV_Mantissa_MUL100  ) * 16 ) / 100

#define usartDIV(_PCLK_, _BAUD_)							(uint32_t) (_PCLK_/(16 * _BAUD_ ))
#define usartDIV_MUL100(_PCLK_, _BAUD_)						(uint32_t) ( (25 * _PCLK_ ) / (4  * _BAUD_ ))
#define Mantissa_MUL100(_PCLK_, _BAUD_)						(uint32_t) (usartDIV(_PCLK_, _BAUD_) * 100)
#define Mantissa(_PCLK_, _BAUD_)							(uint32_t) (usartDIV(_PCLK_, _BAUD_) )
#define DIV_Fraction(_PCLK_, _BAUD_)						(uint32_t) (((usartDIV_MUL100(_PCLK_, _BAUD_) -  Mantissa_MUL100(_PCLK_, _BAUD_) ) * 16 ) / 100 )
#define uart_BRR_Register(_PCLK_, _BAUD_)					(( Mantissa (_PCLK_, _BAUD_) ) <<4 )|( (DIV_Fraction(_PCLK_, _BAUD_)) & 0xF )
enum Polling_mechism{
	enable ,
	disable
};
// * =======================================================================================

/*
 *
 *
 * =======================================================================================
 * 							APIs Supported by "MCAL gpio DRIVER"
 * =======================================================================================
 */
/* Initialization/de-initialization functions  **********************************/
void MCAL_uart_Init (usart_TypeDef *usartx, uart_Config* uart_Config);
void MCAL_uart_DeInit (usart_TypeDef *usartx);
void MCAL_uart_gpio_Set_Pins (usart_TypeDef *usartx);
void MCAL_uart_SendData	(usart_TypeDef *usartx, uint16_t *pTxBuffer,enum Polling_mechism PollingEn );
void MCAL_uart_ReceiveData	(usart_TypeDef *usartx, uint16_t *pTxBuffer ,enum Polling_mechism PollingEn );
void MCAL_uart_WAIT_TC (usart_TypeDef *usartx ) ;

#endif /* STM32F429_usart_DRIVER_H_ */
