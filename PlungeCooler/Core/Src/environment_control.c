#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_i2c.h"
#include "environment_control.h"

#define SHT40_ADDRESS (0x44 << 1)
#include <stdint.h>
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

void getCurrentTRH (float *values) {
        HAL_StatusTypeDef ret;
	uint8_t data_tx[1] = {0xFD};
	uint8_t data_rx[6];
	ret = HAL_I2C_Master_Transmit(&hi2c1, SHT40_ADDRESS, data_tx, 1, 1000);
	if ( ret != HAL_OK ) {
	  printf("Error Tx\r\n");
	}
	else{
		//read bytes
		 HAL_Delay(10);
		 ret =  HAL_I2C_Master_Receive(&hi2c1, SHT40_ADDRESS, (uint8_t*)&data_rx, 6,1000);
		 if ( ret != HAL_OK ) {
			 printf("Error Rx\r\n");
		 }
		 else{
			 for(int i = 0; i < 6 ; i++){
				 printf("data_rx[%i] = %u \n",i,data_rx[i]);
			 }
			 float t_ticks = data_rx[0] * 256 + data_rx[1];
			 float rh_ticks = data_rx[3] * 256 + data_rx[4];

			 values[0] = -45 + 175 * t_ticks/65535;
			 values[1] = -6 + 125 * rh_ticks/65535;
		 }
	}
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
