#include "tp.h"
#include "tpl_os.h"

#include "FPU/inc/Fpu.h"
#include "Csm/inc/Csm.h"
#include "math.h"
#include "V2xM/inc/SchM_V2xM.h"
#include "V2xM/inc/V2xM_V2xGn.h"
#include "SPI/inc/PLL.h"
#include "General/Common_Macros.h"
#include "SPI/inc/Interrupt.h"
#include "USART/inc/stm32f429_USART_driver.h"
#include <time.h>

#include "Application/ECUAbstraction_SWC/inc/ECUAbstraction_SWC.h"
#include "Application/GeoMath_SWC/inc/GeoMath_SWC.h"
#include "Application/Poti_SWC/inc/Poti_SWC.h"
#include "Application/VDP_SWC/inc/VDP_SWC.h"
#include "Application/LDM_SWC/inc/LDM_SWC.h"
#include "Application/V2xFac_Facilities_Services_SWC/inc/V2xFac_Facilities_Services_SWC.h"
#include "Application/V2XDenmLogic_SWC/inc/V2XDenmLogic_SWC.h"

#define GPIO_PORTG_BIT_SET_RESET_REG      (*((volatile uint32 *)0x40021818))

/* uart Initialization */
uart_Config usart1CFG;

WEth_ConfigType configuration={1};

extern uint8 nrfFlag;

int flag_geo = 0;
int flag_fac = 0;
int camCounter = 0;
int flag_denm = 0;
int denm_counter = 0;

void Compare_Denm(V2xFac_DenmMessageRootType* message)
{
	static uint8 DENMPathCount =0;

	if(message->denm.location.traces.arr[0].nCount == DENMPathCount)
	{
		if(BIT_IS_CLEAR(GPIO_PORTG_OUTPUT_DATA_REG,14))
		{
			V2X_SET_BIT(GPIO_PORTG_BIT_SET_RESET_REG,14);
			V2X_CLEAR_BIT(GPIO_PORTG_BIT_SET_RESET_REG,30);
		}
		else
		{
			V2X_SET_BIT(GPIO_PORTG_BIT_SET_RESET_REG,30);
			V2X_CLEAR_BIT(GPIO_PORTG_BIT_SET_RESET_REG,14);
		}
	}

	if(DENMPathCount == 40)
	{
		DENMPathCount =1;
	}
	else{
		DENMPathCount += 1;
	}

}

void Compare_Cam(V2xFac_CamMessageRootType* message)
{
	static uint8 CAPathCount =0;
	camCounter++;

	if(message->coopAwareness.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.pathHistory.nCount == CAPathCount)
	{
		if(BIT_IS_CLEAR(GPIO_PORTG_OUTPUT_DATA_REG,13))
		{
			V2X_SET_BIT(GPIO_PORTG_BIT_SET_RESET_REG,13);
			V2X_CLEAR_BIT(GPIO_PORTG_BIT_SET_RESET_REG,29);
		}
		else
		{
			V2X_SET_BIT(GPIO_PORTG_BIT_SET_RESET_REG,29);
			V2X_CLEAR_BIT(GPIO_PORTG_BIT_SET_RESET_REG,13);
		}
	}

	if(CAPathCount == 40)
	{
		CAPathCount =1;
	}
	else{
		CAPathCount += 1;
	}
}

#define APP_Task_Task_prescan_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
	initBoard();
	Timer4_Init();
	PLL_init();

	Interrupt_Config();
	MCAL_uart_gpio_Set_Pins(usart1);

	usart1CFG.BaudRate = uart_BaudRate_115200;
	usart1CFG.HwFlowCtl = uart_HwFlowCtl_NONE;
	usart1CFG.IRQ_Enable = uart_IRQ_Enable_NONE;
	usart1CFG.P_IRQ_CallBack = NULL;
	usart1CFG.Parity = uart_Parity__NONE;
	usart1CFG.Payload_Length = uart_Payload_Length_8B;
	usart1CFG.StopBits = uart_StopBits__1;
	usart1CFG.usart_Mode = uart_MODE_TX_RX;

	MCAL_uart_Init(usart1, &usart1CFG);

	UpdatePotiData_runnable();
	UpdateVdpData_runnable();
	UpdatePosTime_runnable();

	EthIf_Init(&ethif_congif);
	NvM_Init();
	Crypto_Init(NULL_PTR);
	CryIf_Init(NULL_PTR);
	Csm_Init(NULL_PTR);
	V2xM_Init(NULL_PTR);
	V2xGn_Init((void*)0);
	V2xFac_Init(NULL_PTR);
	V2xBtp_Init((void*)0);
	Gpio_init();
	Spi_Init(&Spi_Configuration);
	V2X_SET_BIT(*(volatile uint32 *)((volatile uint8 *)SPI3_BASE_ADDRESS + SPI_CR1_OFFSET) ,SPI_CR1_MSTR );
	WEth_Init(&configuration);
	Fpu_Init();
	NRF24_RxMode(RxAddress,	channel_value);

	// V2xFac_V2xM_SetCaBsOperation(V2XFAC_CAM_NOT_ACTIVATED);  /* Disable Sending CAM */

	StartOS(OSDEFAULTAPPMODE);
	return 0;
}

TASK(Task_prescan)
{
	UpdatePotiData_runnable();
	UpdateVdpData_runnable();
	UpdatePosTime_runnable();

	TerminateTask();
}
#define APP_Task_Task_prescan_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Task_receive_START_SEC_CODE
#include "tpl_memmap.h"

TASK(Task_receive)
{
	while (1)
	{
		V2xGn_MainFunction();

		Crypto_MainFunction();
		Crypto_MainFunction();

		V2xM_MainFunction();
		V2xM_MainFunction();

		V2xFac_DenBs_MainFunction();
		V2xFac_CaBs_MainFunction();

		DenmMessage_runnable();
		CamMessage_runnable();
		updateCurrentITSStatus_runnable();

		UpdatePotiData_runnable();
		UpdateVdpData_runnable();
		UpdatePosTime_runnable();

		TakeAction_runnable();
	}
}

#define APP_Task_Task_receive_STOP_SEC_CODE
#include "tpl_memmap.h"

unsigned long long  Delay_Counter = 0 ;

#define APP_ISR_isr_tim4_START_SEC_CODE
#include "tpl_memmap.h"

ISR(isr_tim4)
{
	TIM4_SR  &= ~(1<<UIF);
	Delay_Counter++;
}

#define APP_ISR_isr_tim4_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_ISR_isr_nrf_START_SEC_CODE
#include "tpl_memmap.h"

ISR(isr_nrf)
{
	NVIC_DIS0_REG|=(1<<6);
	nrfFlag = 1;

	EthIf_MainFunctionRx();
	if (EXTI_PR & (1<<0))    // If the PA0 triggered the interrupt
	{
		// flag = 1; code

		EXTI_PR|= (1<<0);  // Clear the interrupt flag by writing a 1
	}

	NVIC_EN0_REG |= (1<<6);
}

#define APP_ISR_isr_nrf_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
/*
 *  * This is necessary for ST libraries
 *   */
FUNC(void, OS_CODE) assert_failed(uint8_t* file, uint32_t line)
{
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"

