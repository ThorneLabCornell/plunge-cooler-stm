/*
 * globals.c
 *
 *  Created on: Oct 27, 2023
 *      Author: Leo
 */
#include "globals.h"
#include "main.h"
#include <stdio.h>

void dispense(void) {
//	char a[] = "DEPOSITING!!!\r\n";
//	HAL_UART_Transmit(&huart3, (uint8_t*)a, strlen(a), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(DROP_GPIO_Port, DROP_Pin, 1); //Dispense drop
	for(int i=0; i<20000; i+=2) i--; 		//pseudo-delay. replace with a short timer setup is ideal
	HAL_GPIO_WritePin(DROP_GPIO_Port, DROP_Pin, 0); //Dispense drop

	DEPOSITED = 1;
}

