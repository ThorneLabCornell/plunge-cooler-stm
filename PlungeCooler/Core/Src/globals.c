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
	DEPOSITED = 1;
}
