#include "tp.h"
#include "tpl_os.h"
#include "inc/stm32f429_USART_driver.h"

unsigned char ch = 'a';

/* Dummy Data for testing without HW */
/* Global Variable for holding all received data from prescan */
VehicleVariables receivedData;


/* Global Variable for Poti data */
PotiDataType prescanPotiData = {};

/* Define some constants */
#define NUMBER_OF_VARIABLES 	14
/* Number of variables * size of float + 2 bytes for terminating characters */
#define BUFFER_SIZE_IN_BYTES 	(NUMBER_OF_VARIABLES * 8 + 2)

typedef enum{
	ALT_000_01,
	ALT_000_02,
	ALT_000_05,
	ALT_000_10,
	ALT_000_20,
	ALT_000_50,
	ALT_001_00,
	ALT_002_00,
	ALT_005_00,
	ALT_010_00,
	ALT_020_00,
	ALT_050_00,
	ALT_100_00,
	ALT_200_00,
	ALT_OUTOFRANGE,
	ALT_UNAVAILABLE
}AltitudeConfidenceType;

typedef struct{
	double UTC_TimeStamp;
	double TAI_TimeStamp;
	double latitude;
	double longitude;
	double altitude;
	AltitudeConfidenceType altitudeConfidence;
	double heading;
	uint8 headingConfidence;
	double speed;
	uint8 speedConfidence;
	uint16 semiMajorConfidence;
	uint16 semiMinorConfidence;
	uint16 semiMajorOrientation;
}PotiDataType;

/* Define required data types*/
/* Data type holds all needed variables that sent from Prescan Simulink*/
typedef struct vehicleData
{
	/* Poti data */
	/* X,Y*/
	double x;
	double y;
	/* Time */
	double UTC_TimeStamp;
	double TAI_TimeStamp;
	/* Position */
	double latitude;
	double longitude;
	double altitude;
	/* Dynamics*/
	double velocity;
	double heading;
	double yawRate;
	/* Switches */
	double exteriorLightsValue;
	double lightBarSirenInUseValue;
	/* Vehicle specs */
	double vehicleLength;
	double vehicleWidth;
}vehicleData;

/*****************************************************
 * Runnable: 	getPrescanData
 * Period:		0.05
 *****************************************************/
void getPrescanData( void )
{
	uint8 ch;
	/* wait unitl receive '\r' then '\n'*/
	do
	{
		MCAL_uart_ReceiveData(usart1, (uint16_t*)&ch, enable);
	} while(ch != '\r');
	do
	{
		MCAL_uart_ReceiveData(usart1, (uint16_t*)&ch, enable);
	} while(ch != '\n');

	for(int i = 0; i < BUFFER_SIZE_IN_BYTES; i++)
	{
		MCAL_uart_ReceiveData(usart1, (uint16_t*)(receivedData.bytes + i), enable);
	}
//	for(int i = 0; i < BUFFER_SIZE_IN_BYTES; i++)
//	{
//		MCAL_uart_SendData(usart1, (uint16_t*)(receivedData.bytes + i), enable);
//	}
	/* Update POTI Global data variable with received data */
	prescanPotiData.UTC_TimeStamp = receivedData.data.UTC_TimeStamp;
	prescanPotiData.TAI_TimeStamp = receivedData.data.TAI_TimeStamp;
	prescanPotiData.latitude = receivedData.data.latitude;
	prescanPotiData.longitude = receivedData.data.longitude;
	prescanPotiData.altitude = receivedData.data.altitude;
	prescanPotiData.heading = receivedData.data.heading;
	prescanPotiData.speed = receivedData.data.velocity;
}

/* Data Structure to receive data serially from uart and manipulate data easy in code */
typedef union {
  uint8 bytes[BUFFER_SIZE_IN_BYTES];
  vehicleData data;
}VehicleVariables;

FUNC(int, OS_APPL_CODE) main(void)
{
  initBoard();
  uart_Config usart1CFG;
	MCAL_uart1_gpio_Set_Pins(usart1);

	usart1CFG.BaudRate = uart_BaudRate_115200;
	usart1CFG.HwFlowCtl = uart_HwFlowCtl_NONE;
	usart1CFG.IRQ_Enable = uart_IRQ_Enable_NONE;
	usart1CFG.P_IRQ_CallBack = NULL;
	usart1CFG.Parity = uart_Parity__NONE;
	usart1CFG.Payload_Length = uart_Payload_Length_8B;
	usart1CFG.StopBits = uart_StopBits__1;
	usart1CFG.usart_Mode = uart_MODE_TX_RX;

	MCAL_uart_Init(usart1, &usart1CFG);



  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

#define APP_Task_blink_START_SEC_CODE
#include "tpl_memmap.h"

TASK(blink)
{
  while(1) {
    MCAL_uart_ReceiveData	(usart1, (uint16_t *)&ch,enable);
    ledToggle(RED);
    MCAL_uart_SendData	(usart1, (uint16_t *)&ch,enable);
  }
}

#define APP_Task_blink_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
/*
 *  * This is necessary for ST libraries
 *   */
FUNC(void, OS_CODE) assert_failed(uint8_t* file, uint32_t line)
{
}

FUNC(void, OS_CODE) PreTaskHook()
{
  TaskType task_id = 0;
  GetTaskID(&task_id);
}

FUNC(void, OS_CODE) PostTaskHook()
{
  TaskType task_id = 0;
  GetTaskID(&task_id);
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"

