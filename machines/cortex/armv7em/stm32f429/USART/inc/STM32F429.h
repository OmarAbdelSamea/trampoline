#ifndef STM32F103X8_H_
#define STM32F103X8_H_

//-----------------------------
//Includes
//-----------------------------

#include "stdlib.h"
#include <stdint.h>


//-----------------------------
//Base addresses for Memories
//-----------------------------
#define FLASH_Memory_BASE            					0x08000000UL
#define System_Memory_BASE            					0x1FFFF000UL
#define sRAM_BASE            							0x20000000UL

#define Peripherals_BASE            					0x40000000UL

#define Cortex_M3_Internal_Peripherals_BASE            	0xE0000000UL
//nvic register map ***************************************************
#define nvic_Base					(0xE000E100UL)
#define nvic_ISER0					*(volatile uint32_t *) (nvic_Base + 0x0 )
#define nvic_ISER1					*(volatile uint32_t *)(nvic_Base + 0x4)
#define nvic_ISER2					*(volatile uint32_t *)(nvic_Base + 0x8)
#define nvic_ICER0					*(volatile uint32_t *)(nvic_Base + 0x80)
#define nvic_ICER1					*(volatile uint32_t *)(nvic_Base + 0x84)
#define nvic_ICER2					*(volatile uint32_t *)(nvic_Base + 0x88)
//-----------------------------
//Base addresses for AHB Peripherals
//-----------------------------
#define rcc_BASE              (Peripherals_BASE + 0x00023800UL)

//-----------------------------
//Base addresses for APB2 Peripherals
//-----------------------------

//gpio
//A,B fully included in LQFP48 Package
#define gpioA_BASE            (Peripherals_BASE + 0x00020000UL)
#define gpioB_BASE            (Peripherals_BASE + 0x00020400UL)
//C,D Partial  included in LQFP48 Package
#define gpioC_BASE            (Peripherals_BASE + 0x00020800UL)
#define gpioD_BASE            (Peripherals_BASE + 0x00020C00UL)
//EP not  included in LQFP48 Package
#define gpioE_BASE            (Peripherals_BASE + 0x00021000UL)
//-------
#define usart1_BASE           (Peripherals_BASE + 0x00011000UL)
#define usart6_BASE           (Peripherals_BASE + 0x00011400UL)


//-----------------------------
//Base addresses for APB1 Peripherals
//-----------------------------
#define usart2_BASE             (Peripherals_BASE + 0x00004400UL)
#define usart3_BASE             (Peripherals_BASE + 0x00004800UL)


//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//Peripheral register
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: gpio
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
} gpio_TypeDef;

//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: rcc
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t none1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t none2;
	volatile uint32_t none3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t none4;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t none5;
	volatile uint32_t none6;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t none7;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t none8;
	volatile uint32_t none9;
	volatile uint32_t BDCR;
	volatile uint32_t gCSR;
	volatile uint32_t none10;
	volatile uint32_t none11;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t none12;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t none13;
	volatile uint32_t none14;
	volatile uint32_t none15;
	volatile uint32_t PDCKCFGR;
} rcc_TypeDef;

//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: usart
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} usart_TypeDef;



//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*


//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:
//-*-*-*-*-*-*-*-*-*-*-*

#define gpioA               ((gpio_TypeDef *)gpioA_BASE)
#define gpioB               ((gpio_TypeDef *)gpioB_BASE)
#define gpioC               ((gpio_TypeDef *)gpioC_BASE)
#define gpioD               ((gpio_TypeDef *)gpioD_BASE)
#define gpioE               ((gpio_TypeDef *)gpioE_BASE)

#define rcc                 ((rcc_TypeDef *)0x40023800)

#define usart1                ((usart_TypeDef *)usart1_BASE)
#define usart2                ((usart_TypeDef *)usart2_BASE)
#define usart3                ((usart_TypeDef *)usart3_BASE)
#define usart6                ((usart_TypeDef *)usart6_BASE)


//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//-*-*-*-*-*-*-*-*-*-*-*-
//clock enable Macros:
//-*-*-*-*-*-*-*-*-*-*-*

//Enable clock gpioA
#define rcc_gpioA_CLK_EN()	( rcc->AHB1ENR |= (1<<0) )
//Enable clock gpioB
#define rcc_gpioB_CLK_EN()	( rcc->AHB1ENR |= (1<<1) )
#define rcc_gpioC_CLK_EN()	( rcc->AHB1ENR |= (1<<2) )
#define rcc_gpioD_CLK_EN()	( rcc->AHB1ENR |= (1<<3) )
#define rcc_gpioE_CLK_EN()	( rcc->AHB1ENR |= (1<<4) )

#define rcc_usart1_CLK_EN()	( rcc->APB2ENR |= (1<<4) )
#define rcc_usart2_CLK_EN()	( rcc->APB1ENR |= (1<<17) )
#define rcc_usart3_CLK_EN()	( rcc->APB1ENR |= (1<<18) )
#define rcc_usart6_CLK_EN()	( rcc->APB2ENR |= (1<<5) )


//rcc_Reset
#define rcc_usart1_Reset()	( rcc->APB2RSTR |= (1<<14) )
#define rcc_usart2_Reset()	( rcc->APB1RSTR |= (1<<17) )
#define rcc_usart3_Reset()	( rcc->APB1RSTR |= (1<<18) )

//-*-*-*-*-*-*-*-*-*-*-*-
//IVT
//-*-*-*-*-*-*-*-*-*-*-*
#define 	usart1_IRQ			37
#define 	usart2_IRQ			38
#define 	usart3_IRQ			39
#define 	usart6_IRQ			71

//-*-*-*-*-*-*-*-*-*-*-*-
//nvic IRQ enable/Disable Macros:
//-*-*-*-*-*-*-*-*-*-*-*
//usart
#define nvic_IRQ37_usart1_Enable   	(nvic_ISER1 |= 1<<( usart1_IRQ - 32 )) //IRQ-32
#define nvic_IRQ38_usart2_Enable   	(nvic_ISER1 |= 1<<( usart2_IRQ - 32 )) //IRQ-32
#define nvic_IRQ39_usart3_Enable   	(nvic_ISER1 |= 1<<( usart3_IRQ - 32 )) //IRQ-32
#define nvic_IRQ39_usart6_Enable   	(nvic_ISER1 |= 1<<( usart6_IRQ - 32 )) //IRQ-32

#define nvic_IRQ37_usart1_Disable   	(nvic_ICER1 |= 1<<( usart1_IRQ- 32 )) //IRQ-32
#define nvic_IRQ38_usart2_Disable   	(nvic_ICER1 |= 1<<( usart2_IRQ- 32 )) //IRQ-32
#define nvic_IRQ39_usart3_Disable   	(nvic_ICER1 |= 1<<( usart3_IRQ- 32 )) //IRQ-32
#define nvic_IRQ39_usart6_Disable   	(nvic_ICER1 |= 1<<( usart6_IRQ- 32 )) //IRQ-32


#endif /* STM32F103X8_H_ */
