/*
 * Algorithm.c
 *
 *  Created on: Mar 5, 2022
 *      Author: Salma Sherif
 */

#include "SWC1_Led_Test/Rte_SWC1_Led_Test.h"


/*implementation of runnable (called in OS) s*/
void blink_led(){

	uint8 flag = 1;
	Rte_Write_SWC1_Led_Test_PP_FlagLed_Flag(flag);
}
