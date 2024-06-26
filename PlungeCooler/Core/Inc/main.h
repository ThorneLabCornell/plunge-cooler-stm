/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/*** MOTOR CONTROL FUNCTIONS ***/
void move_tilt_steps(uint32_t delay, uint8_t dir, uint32_t num_steps);
void move_tilt_deg(uint32_t degrees, uint8_t dir);
void move_pan_steps(uint32_t delay, uint8_t dir, uint32_t num_steps);
void move_pan_deg(uint32_t degrees, uint8_t dir);

void step(uint32_t delay, uint32_t port, uint32_t pin, uint8_t dir);
void move_dispenser(void);
/*** USART Rx HANDLE ***/
void ack(void);
void bad(void);

/*** PLUNGE ***/
void start_plunge(void);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOF
#define PH0_MCU_Pin GPIO_PIN_0
#define PH0_MCU_GPIO_Port GPIOH
#define PH1_MCU_Pin GPIO_PIN_1
#define PH1_MCU_GPIO_Port GPIOH
#define PAN_EN_Pin GPIO_PIN_0
#define PAN_EN_GPIO_Port GPIOC
#define TILT_EN_Pin GPIO_PIN_2
#define TILT_EN_GPIO_Port GPIOC
#define TILT_STP_Pin GPIO_PIN_3
#define TILT_STP_GPIO_Port GPIOC
#define PLUNGE_ENC_A_Pin GPIO_PIN_0
#define PLUNGE_ENC_A_GPIO_Port GPIOA
#define PLUNGE_ENC_B_Pin GPIO_PIN_1
#define PLUNGE_ENC_B_GPIO_Port GPIOA
#define PAN_DIR_Pin GPIO_PIN_3
#define PAN_DIR_GPIO_Port GPIOA
#define BRAKE_Pin GPIO_PIN_6
#define BRAKE_GPIO_Port GPIOA
#define TILT_DIR_Pin GPIO_PIN_1
#define TILT_DIR_GPIO_Port GPIOB
#define PAN_STP_Pin GPIO_PIN_11
#define PAN_STP_GPIO_Port GPIOF
#define DROP_Pin GPIO_PIN_10
#define DROP_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define USB_FS_N_Pin GPIO_PIN_11
#define USB_FS_N_GPIO_Port GPIOA
#define USB_FS_P_Pin GPIO_PIN_12
#define USB_FS_P_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define ENC_DMA_POLL_FREQ 1000

#define PULSES_TO_DEGREES 200

#define PAN_DEFAULT_DELAY 1
#define TILT_DEFAULT_DELAY 1
#define DIR_PAN_LEFT 1
#define DIR_PAN_RIGHT 0
#define DIR_TILT_UP 1
#define DIR_TILT_DOWN 0
#define PAN_SWITCH_POS 1000
#define TILT_SWITCH_POS 1000
#define PAN_DEG_TO_STEPS 200*64
#define TILT_DEG_TO_STEPS 200*64

#define ACK 'Z'
#define BAD 'X'

#define MOVE '1'
#define PLUNGE '2'
#define STATUS '3'
#define RELEASE '4'

#define UP '1'
#define DOWN '2'
#define LEFT '3'
#define RIGHT '4'


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
