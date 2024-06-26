/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
// test 12
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "stm32h7xx_it.h"
#include "globals.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*** GLOBAL VARS ***/
uint16_t posLog[LOG_SIZE] = {0};
uint32_t log_position = 0;
uint32_t running_sum = 0;
uint32_t timepoint_pos = 0;
uint32_t dispense_delay_clocks = 0;
uint8_t  disp_flag = 0;
uint32_t dispense_pos = 0;
uint32_t posn_braked_at = 0;
uint8_t  plunge_done_flag = 0;
uint8_t  DEPOSITED = 0;
uint32_t clocks_to_disp = 0;


uint8_t val = 1;

uint8_t rxBuffer[100];

uint8_t uartRxBuffer[100];
uint16_t rxIndex = 0;
uint8_t received_character[1] = {0};

uint8_t tx_ack[3] = {ACK, '\r', '\n'};
uint8_t tx_bad[3] = {BAD, '\r', '\n'};
uint8_t rx_flag = 0;
int panPos = 0;
int tiltPos = 0;

/*** MOTOR CONTROL FUNCTIONS ***/
void move_tilt_steps(uint32_t delay, uint8_t dir, uint32_t num_steps) {
	HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, 0);
	HAL_GPIO_WritePin(TILT_DIR_GPIO_Port, TILT_DIR_Pin, dir);
	for(int i=0; i<num_steps; i++) {
		HAL_GPIO_WritePin(TILT_STP_GPIO_Port, TILT_STP_Pin, GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(TILT_STP_GPIO_Port, TILT_STP_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay);


	}
//	char b[] = "done steps\r\n";
//	HAL_UART_Transmit(&huart3, (uint8_t*)b, strlen(b), HAL_MAX_DELAY);

	tiltPos += num_steps * (1 - 2 * dir); // + if dir is 0, else -
//	HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, 1);

}

void move_tilt_deg(uint32_t degrees, uint8_t dir) {
	move_tilt_steps(TILT_DEFAULT_DELAY, dir, degrees*TILT_DEG_TO_STEPS);
}

void move_pan_steps(uint32_t delay, uint8_t dir, uint32_t num_steps) {
	HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, 0);
	HAL_GPIO_WritePin(PAN_DIR_GPIO_Port, PAN_DIR_Pin, dir);
	for(int i=0; i<num_steps; i++) {
		HAL_GPIO_WritePin(PAN_STP_GPIO_Port, PAN_STP_Pin, GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(PAN_STP_GPIO_Port, PAN_STP_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay);
	}
//	char b[] = "done steps\r\n";
//	HAL_UART_Transmit(&huart3, (uint8_t*)b, strlen(b), HAL_MAX_DELAY);

	panPos += num_steps * (1 - 2 * dir); // + if dir is 0, else -
//	HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, 1);

}

void move_pan_deg(uint32_t degrees, uint8_t dir) {
	move_pan_steps(PAN_DEFAULT_DELAY, dir, degrees*PAN_DEG_TO_STEPS);
	char pos[30];
//	sprintf(pos, "panPos: %d\r\n", panPos);
//	HAL_UART_Transmit(&huart3, (uint8_t*)pos, strlen(pos), HAL_MAX_DELAY);

}

void move_dispenser(void) {


}

/*** PLUNGE FUNCTIONALITY ***/
void start_plunge(void) {

}

/*** USART Rx HANDLE ***/
void ack(void) {
    HAL_UART_Transmit(&huart3, tx_ack, 3, HAL_MAX_DELAY);
}

void bad(void) {
	HAL_UART_Transmit(&huart3, tx_bad, 3, HAL_MAX_DELAY);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    rxBuffer[rxIndex] = received_character[0];
	rxIndex++;

	if (received_character[0] == '\n' || received_character[0] == '\r') {
    	rxIndex = 0;
    	rx_flag = 1;
    } else {
        HAL_UART_Receive_IT(&huart3, received_character, 1);
    }
}

void rx_handle(void) {
	HAL_GPIO_WritePin(GPIOE, LD2_Pin, val);
	if(val)
		val = 0;
	else
		val = 1;
	//sprintf(num, "RX0: %d\r\n", rxBuffer[0]);
	//HAL_UART_Transmit(&huart3, (uint8_t*)num, strlen(num), HAL_MAX_DELAY);
	switch(rxBuffer[0]) {
    	case MOVE: ;
    		uint32_t amount = 0;
			for(int i=2; i<=4; i++) {
				 char digit = rxBuffer[i];
				 if (digit >= '0' && digit <= '9') {
					 amount = (amount * 10) + (digit - '0');	// Shift the existing result left by one decimal place and add the digit value
				 }
			}

			char response[100];
//			sprintf(response, "%c%c received this amount: %d\r\n", (int)rxBuffer[0], (int)rxBuffer[1], (int)amount);
//			HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);

			switch(rxBuffer[1]) {
				case UP: ;
					move_tilt_deg(amount, DIR_TILT_UP);
					ack();
					break;
				case DOWN: ;
					move_tilt_deg(amount, DIR_TILT_DOWN);
					ack();
					break;
				case LEFT: ;
					move_pan_deg(amount, DIR_PAN_LEFT);
					ack();
					break;
				case RIGHT: ;
					move_pan_deg(amount, DIR_PAN_RIGHT);
					ack();
					break;
				default: ;
					bad();
					break;
			}

			break;

		case PLUNGE: ;
			HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, 1); //disengage brake
			HAL_UART_Transmit(&huart3, tx_ack, sizeof(tx_ack), HAL_MAX_DELAY);

		/* retrieve info */
			plunge_done_flag = 0;

			uint32_t brake_pos = 0;
			timepoint_pos = 0;
			for(int i=1; i<=6; i++) {
				 char digit = rxBuffer[i];
				 if (digit >= '0' && digit <= '9') {
					 brake_pos = (brake_pos * 10) + (digit - '0');	// Shift the existing result left by one decimal place and add the digit value
				 }
			}
			for(int i=7; i<=12; i++) {
				 char digit = rxBuffer[i];
				 if (digit >= '0' && digit <= '9') {
					 timepoint_pos = (timepoint_pos * 10) + (digit - '0');	// Shift the existing result left by one decimal place and add the digit value
				 }
			}

//			char num[30];
//			sprintf(num, "brake_pos: %d, timepoint_pos: %d\r\n", brake_pos, timepoint_pos);
	//		HAL_UART_Transmit(&huart3, (uint8_t*)num, strlen(num), HAL_MAX_DELAY);

			/***TODO: figure out what angle to tilt to given timepoint_pos***/

			/* reset tracking variables */
			log_position = 0;
			running_sum = 0;
			memset(posLog, 0, sizeof(posLog));
			DEPOSITED = 0;

			/* dispense position calculation */
			//						    us								mm						mm/s	   s->us
			dispense_delay_clocks = 10000;//(DISPENSE_LATENCY + (TARGET_DIST_ORTH*cos(tiltPos*M_PI/180))/DROP_SPEED*1000*1000)*US_TO_TICKS;
			// ^ delay in ticks between dispensing and drop hit target in x direction


			/* configure tim4 for final dispense timing */
			TIM4->CR1  &= ~TIM_CR1_CEN;

			TIM4->CNT   =  100;				// 100 included here and in ARR to make sure it doesnt immediately underflow if it vibrates up
			TIM4->ARR 	= brake_pos; 	// Counter rolls over at brake_pos which triggers an interrupt handled in TIM2_IRQHandler (stm32h7xx_it.c)
			TIM4->SR   &= ~TIM_SR_UIF; 		// Clear the interrupt flag
			TIM4->CR1  &= ~TIM_CR1_UDIS;	// make sure update is enabled
			TIM4->DIER |=  TIM_DIER_UIE; 	// update interrupt enabled
			TIM4->CR1  |= TIM_CR1_ARPE;		// enable auto reload preload

			/* configuring encoder counter */
			TIM2->CR1  &= ~TIM_CR1_CEN;

			TIM2->CNT   =  100;				// 100 included here and in ARR to make sure it doesnt immediately underflow if it vibrates up
			TIM2->ARR 	= brake_pos; 		// Counter rolls over at brake_pos which triggers an interrupt handled in TIM2_IRQHandler (stm32h7xx_it.c)
			TIM2->SR   &= ~TIM_SR_UIF; 		// Clear the interrupt flag
			TIM2->CR1  &= ~TIM_CR1_UDIS;	// make sure update is enabled
			TIM2->DIER |=  TIM_DIER_UIE; 	// update interrupt enabled
			TIM2->CR1  |= TIM_CR1_ARPE;		// enable auto reload preload

			TIM2->CR1  |=  TIM_CR1_CEN; 	//start counter

			/* configuring data logging timer */
			TIM5->CR1  &= ~TIM_CR1_CEN; // Start TIM5 to commence data collection

			TIM5-> CNT  = 100;				//
			TIM5->ARR 	= CLOCKS_PER_LOG; 	// check positione interval
			TIM5->CR1  &= ~TIM_CR1_UDIS;	// make sure update is enabled
			TIM5->DIER |=  TIM_DIER_UIE; 	// update interrupt enabled
			TIM5->CR1  |= TIM_CR1_ARPE;		// enable auto reload preload
			TIM5->SR   &= ~TIM_SR_UIF; 		// Clear the interrupt flag

			TIM5->CR1  |= TIM_CR1_CEN; // Start TIM5 to commence data collection

			/* for debug, transmit encoder position */
//			uint32_t enc_pos;
//			for(int i=0; i<200; i++) {
//				char response[100] = {0};
//				enc_pos = TIM2->CNT;
//				sprintf(response, "enc: %d; disp_f: %d; runsum: %d; tim4: %d; log_pos: %d, clocks_to_disp: %d; disp: %d\r\n", (int)enc_pos, (int)disp_flag, (int)running_sum, (int)TIM4->CNT, (int)log_position, (int)TIM4->ARR, (int)DEPOSITED);
//				HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
//			}
			break;

		case RELEASE: ;
			HAL_UART_Transmit(&huart3, tx_ack, sizeof(tx_ack), HAL_MAX_DELAY);

			HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, 1); //disengage brake
//			char j[100] = {0};
//			sprintf(j, "RELEASE\r\n");
//			HAL_UART_Transmit(&huart3, (uint8_t*)j, strlen(j), HAL_MAX_DELAY);

			break;
		case '5': ;
			HAL_UART_Transmit(&huart3, tx_ack, sizeof(tx_ack), HAL_MAX_DELAY);

			HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, 0); //engage brake
//			char k[100] = {0};
//			sprintf(k, "BRAKE\r\n");
//			HAL_UART_Transmit(&huart3, (uint8_t*)k, strlen(k), HAL_MAX_DELAY);

			break;
		case '6': ;
			char lm[100] = {0};
			//sprintf(lm, "current position: %d\r\n braked at: %d\r\n", TIM2->CNT, posn_braked_at);
			//HAL_UART_Transmit(&huart3, (uint8_t*)lm, strlen(lm), HAL_MAX_DELAY);
			break;

    }
//	char b[] = "done handling\r\n";
//	HAL_UART_Transmit(&huart3, (uint8_t*)b, strlen(b), HAL_MAX_DELAY);

    rx_flag = 0;
    HAL_UART_Receive_IT(&huart3, received_character, 1);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, received_character, 1); // initialize interrupts

//  char msg[] = "program start \r\n";
//  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  // TIM2 for encoder tick counting
  NVIC_SetPriority(TIM2_IRQn, 1); // Braking is priority as long as dispense isnt happening
  NVIC_EnableIRQ(TIM2_IRQn);

  // TIM5 for logkeeping and speed calculations and TIM4 setup
  NVIC_SetPriority(TIM5_IRQn, 2); // Log keeping should be interruptable
  NVIC_EnableIRQ(TIM5_IRQn);

  // TIM4 for dispense timing after commenced by TIM5
  NVIC_SetPriority(TIM4_IRQn, 0); // Dispense accuracy is top priority
  NVIC_EnableIRQ(TIM4_IRQn);

  HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, 1); // Ensure brake is disengaged after reset
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1) {
	  if(rx_flag) rx_handle();


	  if(plunge_done_flag) {
		  HAL_UART_Transmit(&huart3, tx_ack, sizeof(tx_ack), HAL_MAX_DELAY);
		  char msg[10];
		  for(int i=0 ; i<log_position; i++) {
			  sprintf(msg, "%u\n", posLog[i]);
//			  bytes[0] = ((posLog[i] >> 24) 	& 0xFF);
//			  bytes[1] = ((posLog[i] >> 16) 	& 0xFF);
//			  bytes[2] = ((posLog[i] >> 8) 		& 0xFF);
//			  bytes[3] = ((posLog[i])	 		& 0xFF);
//
//			  HAL_UART_Transmit(&huart3, bytes, 4, HAL_MAX_DELAY);
//			  HAL_UART_Transmit(&huart3, (uint8_t*)rn, strlen(rn), HAL_MAX_DELAY);
			  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		  }
		  HAL_UART_Transmit(&huart3, tx_ack, sizeof(tx_ack), HAL_MAX_DELAY);

		  plunge_done_flag = 0;
	  }
//	  if(DEPOSITED) {
//		  char m[]  = "DEPOSITED\r\n";
//		  HAL_UART_Transmit(&huart3, (uint8_t*)m, strlen(m), HAL_MAX_DELAY);
//		  DEPOSITED = 0;
//	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
	TIM2->CR1  |= TIM_CR1_UDIS;	// make sure update is enabled
	TIM2->DIER &=  ~TIM_DIER_UIE; 	// update interrupt enabled
	TIM2->CR1 &= ~TIM_CR1_CEN;
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
	TIM4->CR1 &= ~TIM_CR1_CEN; 	// dont start it
	TIM4->CNT = 0;				//reset it

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

//  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN; // Enable TIM5 clock
//  TIM5->DIER |= TIM_DIER_UIE; // Enable update interrupt
//  TIM5->ARR = ENC_DMA_POLL_FREQ; // Timebase

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, USB_FS_PWR_EN_Pin|PAN_STP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PAN_EN_Pin|TILT_EN_Pin|TILT_STP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PAN_DIR_Pin|BRAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TILT_DIR_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DROP_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_PWR_EN_Pin PAN_STP_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin|PAN_STP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PAN_EN_Pin TILT_EN_Pin TILT_STP_Pin */
  GPIO_InitStruct.Pin = PAN_EN_Pin|TILT_EN_Pin|TILT_STP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PAN_DIR_Pin */
  GPIO_InitStruct.Pin = PAN_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PAN_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BRAKE_Pin */
  GPIO_InitStruct.Pin = BRAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BRAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TILT_DIR_Pin LD3_Pin */
  GPIO_InitStruct.Pin = TILT_DIR_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DROP_Pin LD2_Pin */
  GPIO_InitStruct.Pin = DROP_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_N_Pin USB_FS_P_Pin */
  GPIO_InitStruct.Pin = USB_FS_N_Pin|USB_FS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
