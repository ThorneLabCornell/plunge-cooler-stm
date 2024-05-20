/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "globals.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
double speed = 0;
uint32_t current_pos = 0;
uint32_t current_temp = 0;
uint32_t prev_pos = 0;
uint32_t next_pos = 0;
uint32_t next_next_pos = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart3;
extern ADC_HandleTypeDef hadc1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
//NOTE TO MORNING LEO: LOG SEEMS TO WORK, SHOULD TRANSMIT IT TO PC FOR EASY VISUALIZATION
	if ((TIM2->SR & TIM_SR_UIF) != 0) { // check if update interrupt occured
		HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, 0); //engage brake
		posn_braked_at = TIM2->CNT;
		//also somehow make sure motor is stopped

		plunge_done_flag = 1;

		TIM2->CR1  |= TIM_CR1_UDIS;	// make sure update is enabled
		TIM2->DIER &=  ~TIM_DIER_UIE; 	// update interrupt enabled
		TIM2->CR1 &= ~TIM_CR1_CEN;		// disable tim2
		TIM5->CR1  &= ~TIM_CR1_CEN; 	// disable tim5

		//TIM2->ARR = 999999;			//massive so that we dont hit it when we try to move around. later i can just diable timer but for debugging i want to preserve tim2->cnt
    }
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	if ((TIM4->SR & TIM_SR_UIF) != 0) {
		DEPOSITED = 1;
		dispense();
		TIM4->DIER &=  ~TIM_DIER_UIE; 	// update interrupt disabled
		TIM4->CR1 &= ~TIM_CR1_CEN; 	// only fire this timer once
	}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	//dispense();
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
		/* always do the logging portion */
		/* TODO: Convert this datalogging to DMA to speed it up. actually a priority I think it would have big performance gains*/
		log_position += 1; // increment number of data points taken
		current_pos = TIM2->CNT;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    current_temp = HAL_ADC_GetValue(&hadc1);
		posLog[log_position] = current_pos; 
    thermoLog[log_position] = current_temp; // update log of positions
		running_sum += current_pos - prev_pos;
		prev_pos = current_pos;
		/* if disp hasnt triggered yet, calculate speed and find disp pos */
		if(!disp_flag){
			if(log_position >= MOVING_AVG_LENGTH) {
				/*update running sum of most recent n data points*/
				running_sum -= (posLog[log_position - MOVING_AVG_LENGTH + 1] - posLog[log_position - MOVING_AVG_LENGTH]);
				speed = (double)running_sum / (double)MOVING_AVG_LENGTH; // speed in pulses per log

				uint32_t clocks_per_encoder_pulse = (CLOCKS_PER_LOG / speed); // a form of speed measurement

				next_next_pos = current_pos + 2*speed;	// predicted position 2 TIM5 updates from now. if this is beyond disp_pos we want to trigger TIM4

				// enc				enc		   enc/log			clocks 				clocks/log
				dispense_pos = (timepoint_pos) - (speed * (dispense_delay_clocks / CLOCKS_PER_LOG));
				//		(enc ticks @ intersection) - (encoder ticks between disp signal and contact )
				// dispense_delay_clocks is based on the geometry of the dipense (ie distance and speed that the drop is shooting at)

				if(next_next_pos > dispense_pos) { // dispense comes within the next timebase
//					TIM5->ARR = TIM5->ARR * POST_DISP_LOG_SLOW_FACTOR; // slow down the logging rate after dispense since it is less essential

					/* Insert a marker for when dispense happened */
					//log_position += 1;
					//posLog[log_position] = 12345678;

					disp_flag = 1;

					// using this with a third timer is likely the most accurate. allows inter-encoder values
					clocks_to_disp = clocks_per_encoder_pulse*(dispense_pos-current_pos);

					/* start TIM4 to count to clocks_to_disp */
					TIM4->CR1  &= ~TIM_CR1_CEN; 	//stop counter
					TIM4->CNT   = 0;				// Reset count
					TIM4->ARR 	= clocks_to_disp; 	// Update event when we want to dispense
					TIM4->SR   &= ~TIM_SR_UIF; 		// Clear the interrupt flag
					TIM4->CR1  &= ~TIM_CR1_UDIS;	// make sure update is enabled
					TIM4->DIER |=  TIM_DIER_UIE; 	// update interrupt enabled
					TIM4->CR1  |=  TIM_CR1_ARPE;	// enable auto reload preload
					TIM4->CR1  |=  TIM_CR1_CEN; 	//start counter

				}
			}


		}

		TIM5->SR &= ~TIM_SR_UIF; // Clear the TIM5 update flag
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
