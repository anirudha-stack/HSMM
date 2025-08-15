/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOC
#define LEFT_ENC_A_Pin GPIO_PIN_0
#define LEFT_ENC_A_GPIO_Port GPIOA
#define LEFT_ENC_B_Pin GPIO_PIN_1
#define LEFT_ENC_B_GPIO_Port GPIOA
#define RIGHT_MOT_DIR_Pin GPIO_PIN_2
#define RIGHT_MOT_DIR_GPIO_Port GPIOA
#define LEFT_MOT_DIR_Pin GPIO_PIN_3
#define LEFT_MOT_DIR_GPIO_Port GPIOA
#define SENSOR_1_Pin GPIO_PIN_4
#define SENSOR_1_GPIO_Port GPIOA
#define SENSOR_2_Pin GPIO_PIN_5
#define SENSOR_2_GPIO_Port GPIOA
#define SENSOR_3_Pin GPIO_PIN_6
#define SENSOR_3_GPIO_Port GPIOA
#define SENSOR_4_Pin GPIO_PIN_7
#define SENSOR_4_GPIO_Port GPIOA
#define EMIT_3_Pin GPIO_PIN_0
#define EMIT_3_GPIO_Port GPIOB
#define EMIT_4_Pin GPIO_PIN_1
#define EMIT_4_GPIO_Port GPIOB
#define USER_BUTT_Pin GPIO_PIN_10
#define USER_BUTT_GPIO_Port GPIOD
#define EMIT_1_Pin GPIO_PIN_6
#define EMIT_1_GPIO_Port GPIOC
#define EMIT_2_Pin GPIO_PIN_7
#define EMIT_2_GPIO_Port GPIOC
#define RIGHT_MOT_PWM_Pin GPIO_PIN_11
#define RIGHT_MOT_PWM_GPIO_Port GPIOA
#define RIGHT_ENC_A_Pin GPIO_PIN_15
#define RIGHT_ENC_A_GPIO_Port GPIOA
#define BLE_TX_Pin GPIO_PIN_10
#define BLE_TX_GPIO_Port GPIOC
#define BLE_RX_Pin GPIO_PIN_11
#define BLE_RX_GPIO_Port GPIOC
#define RIGHT_ENC_B_Pin GPIO_PIN_3
#define RIGHT_ENC_B_GPIO_Port GPIOB
#define SENSOR_SCL_Pin GPIO_PIN_6
#define SENSOR_SCL_GPIO_Port GPIOB
#define SENSOR_SDA_Pin GPIO_PIN_7
#define SENSOR_SDA_GPIO_Port GPIOB
#define LEFT_MOT_PWM_Pin GPIO_PIN_9
#define LEFT_MOT_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
