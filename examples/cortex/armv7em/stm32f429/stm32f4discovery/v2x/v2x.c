#include "tp.h"
#include "tpl_os.h"

#include "FPU/inc/Fpu.h"
#include "Csm/inc/Csm.h"
#include "math.h"
#include "V2xM/inc/SchM_V2xM.h"
#include "V2xM/inc/V2xM_V2xGn.h"
#include "SPI/inc/PLL.h"

#include "usart/inc/stm32f429_usart_driver.h"
#include <time.h>

#include "Application/ECUAbstraction_SWC/inc/ECUAbstraction_SWC.h"
#include "Application/GeoMath_SWC/inc/GeoMath_SWC.h"
#include "Application/Poti_SWC/inc/Poti_SWC.h"
#include "Application/VDP_SWC/inc/VDP_SWC.h"
#include "Application/V2xFac_Facilities_Services_SWC/inc/V2xFac_Facilities_Services_SWC.h"

#define GPIO_PORTG_BIT_SET_RESET_REG      (*((volatile uint32 *)0x40021818))

/* uart Initialization */
uart_Config usart1CFG;

WEth_ConfigType configuration={1};

int flag_geo = 0;
int flag_fac = 0;
int camCounter = 0;
int flag_denm = 0;
int denm_counter = 0;
#define APP_Task_Task_1ms_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
	initBoard();
	Timer4_Init();
	PLL_init();


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
	NRF24_TxMode(TxAddress,channel_value);

	// V2xFac_V2xM_SetCaBsOperation(V2XFAC_CAM_NOT_ACTIVATED);  /* Disable Sending CAM */

	/* Send becon only once */
	UpdatePotiData_runnable();
	UpdateVdpData_runnable();
	UpdatePosTime_runnable();

	V2xGn_MainFunction();

	Crypto_MainFunction();

	V2xM_MainFunction();
	EthIf_MainFunctionTx();
    /***********************/ 

	StartOS(OSDEFAULTAPPMODE);
	return 0;
}

TASK(Task_1ms)
{
	UpdatePotiData_runnable();
	UpdateVdpData_runnable();
	UpdatePosTime_runnable();

	V2xGn_MainFunction();
	// TerminateTask();
}
#define APP_Task_Task_1ms_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Task_50ms_START_SEC_CODE
#include "tpl_memmap.h"

TASK(Task_50ms)
{
	
	Update_Denm_runnable();
	CheckConditionsAndTriggerDENM_runnable();

	V2xFac_DenBs_MainFunction();
	
	Crypto_MainFunction();

	V2xM_MainFunction();
	EthIf_MainFunctionTx();
	// TerminateTask();
}

#define APP_Task_Task_50ms_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Task_100ms_START_SEC_CODE
#include "tpl_memmap.h"

TASK(Task_100ms)
{
	/* Request sending denm if required */

	CAM_Generation_runnable();
	V2xFac_Vdp();
	/* Request sending cam if required */
	V2xFac_CaBs_MainFunction();

	Crypto_MainFunction();

	V2xM_MainFunction();
	EthIf_MainFunctionTx();
	// TerminateTask();
}
#define APP_Task_Task_100ms_STOP_SEC_CODE
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



// /* Offset of the elapsed time since the application strated. */
// uint32 time_Offset;
// uint32 testingTime = 1000;
// extern boolean requestFlag;


// #if (V2xFacStationType == V2XFAC_ST_SPECIALVEHICLES)
// #define INITIAL_TIME 1000

// void updatTestingTime()
// {
// 	testingTime += 10;
// 	time_Offset = testingTime - INITIAL_TIME;
// }
// #endif

// /* uart Initialization */
// uart_Config usart1CFG;

// WEth_ConfigType configuration={1};

// #define APP_Task_all_START_SEC_CODE
// #include "tpl_memmap.h"
// FUNC(int, OS_APPL_CODE) main(void)
// {
// 	initBoard();
// 	Timer4_Init();
// 	PLL_init();


// 	MCAL_uart_gpio_Set_Pins(usart1);

// 	usart1CFG.BaudRate = uart_BaudRate_115200;
// 	usart1CFG.HwFlowCtl = uart_HwFlowCtl_NONE;
// 	usart1CFG.IRQ_Enable = uart_IRQ_Enable_NONE;
// 	usart1CFG.P_IRQ_CallBack = NULL;
// 	usart1CFG.Parity = uart_Parity__NONE;
// 	usart1CFG.Payload_Length = uart_Payload_Length_8B;
// 	usart1CFG.StopBits = uart_StopBits__1;
// 	usart1CFG.usart_Mode = uart_MODE_TX_RX;

// 	MCAL_uart_Init(usart1, &usart1CFG);

// 	UpdatePotiData_runnable();
// 	UpdateVdpData_runnable();
// 	UpdatePosTime_runnable();

// 	EthIf_Init(&ethif_congif);
// 	NvM_Init();
// 	Crypto_Init(NULL_PTR);
// 	CryIf_Init(NULL_PTR);
// 	Csm_Init(NULL_PTR);
// 	V2xM_Init(NULL_PTR);
// 	V2xGn_Init((void*)0);
// 	V2xFac_Init(NULL_PTR);
// 	V2xBtp_Init((void*)0);
// 	Gpio_init();
// 	Spi_Init(&Spi_Configuration);
// 	V2X_SET_BIT(*(volatile uint32 *)((volatile uint8 *)SPI3_BASE_ADDRESS + SPI_CR1_OFFSET) ,SPI_CR1_MSTR );
// 	WEth_Init(&configuration);
// 	Fpu_Init();
// 	NRF24_TxMode(TxAddress,channel_value);
// 	StartOS(OSDEFAULTAPPMODE);
// 	return 0;
// }

// TASK(all)
// {
// 	time_Offset = 0;

// 	ledOn(RED);
// 	while(1)
// 	{

// 		V2xGn_MainFunction();
// 		// if(flag ==1 )
// 		// {
// 			Crypto_MainFunction();
// 			V2xM_MainFunction();
// 			EthIf_MainFunctionTx(); /* beacon Confirmation */
// 			// flag =0;

// //			/* ------------------------ For Denm Testing ----------------------------*/
// //			TriggerEvent(&DenmData, 0, 0, DestinationArea,TrafficClass, &ActionID);
// //			DENM_Counter ++;
// //			DENM_Flag = TRUE;
// 		// }

// 		if((time_Offset % 100 == 0) && (time_Offset >= 100))
// 		{
// 			/*---------------- CAM message test code ------------------- */
// 			CAM_Generation_runnable();
// 			V2xFac_Vdp();
// 			V2xFac_CaBs_MainFunction();
// //			if((time_Offset % 1000 == 0) && (time_Offset >= 1000))
// 			// if (requestFlag == TRUE)
// 			// {
// 				Crypto_MainFunction();
// 				V2xM_MainFunction();
// 				EthIf_MainFunctionTx();
// 				//Delay_MS(200);
// 			// 	requestFlag = FALSE;
// 			// }

// 			/*---------------- DENM message test code ------------------- */
// //			V2xFac_DenBs_MainFunction();
// //			if(DENM_Flag == TRUE)
// //			{
// //				csm_callback();
// //				V2xM_MainFunction();
// //				EthIf_MainFunctionTx();
// //				Delay_MS(200);
// //				DENM_Flag = FALSE;
// //			}
// 		}

// 		/*---------------- DENM message test code ------------------- */
// //		if((time_Offset % 250 == 0) && (time_Offset >= 250))
// //		{
// //			if(DENM_Counter < 70)
// //			{
// //				fill_DENM(&DenmData);
// //				UpdateEvent(&DenmData, 0, 0, DestinationArea,TrafficClass, &ActionID);
// //				DENM_Counter ++;
// //				DENM_Flag = TRUE;
// //			}
// //			else if(DENM_Counter == 70)
// //			{
// //				fill_DENM(&DenmData);
// //				DenmData.management.termination = 1;
// //				DenmData.management.exist.termination = 1;
// //				TerminateEvent(&DenmData, 0, 0, DestinationArea,TrafficClass, &ActionID);
// //				DENM_Counter ++;
// //				DENM_Flag = TRUE;
// //			}
// //		}

// 		UpdatePotiData_runnable();
// 		UpdateVdpData_runnable();
// 		UpdatePosTime_runnable();
// //		time_Offset = RefrenceTime - INITIAL_TIME;
// 		updatTestingTime();
		
// 	}
// }

// #define APP_Task_all_STOP_SEC_CODE
// #include "tpl_memmap.h"


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

