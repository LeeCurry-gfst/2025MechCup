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
#define DIR_Pin GPIO_PIN_7
#define DIR_GPIO_Port GPIOF
#define MS1_Pin GPIO_PIN_8
#define MS1_GPIO_Port GPIOF
#define MS2_Pin GPIO_PIN_9
#define MS2_GPIO_Port GPIOF
#define EN_Pin GPIO_PIN_10
#define EN_GPIO_Port GPIOF
#define STEP_Pin GPIO_PIN_1
#define STEP_GPIO_Port GPIOA
#define POUROUT_Pin GPIO_PIN_6
#define POUROUT_GPIO_Port GPIOA
#define DROP_Pin GPIO_PIN_7
#define DROP_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_5
#define FAN_GPIO_Port GPIOC
#define reserve_Pin GPIO_PIN_0
#define reserve_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_12
#define KEY1_GPIO_Port GPIOF
#define KEY2_Pin GPIO_PIN_15
#define KEY2_GPIO_Port GPIOF
#define KEY3_Pin GPIO_PIN_0
#define KEY3_GPIO_Port GPIOG
#define KEY4_Pin GPIO_PIN_1
#define KEY4_GPIO_Port GPIOG
#define KEY5_Pin GPIO_PIN_7
#define KEY5_GPIO_Port GPIOE
#define KEY6_Pin GPIO_PIN_8
#define KEY6_GPIO_Port GPIOE
#define ARM1_Pin GPIO_PIN_9
#define ARM1_GPIO_Port GPIOE
#define KEY7_Pin GPIO_PIN_10
#define KEY7_GPIO_Port GPIOE
#define ARM2_Pin GPIO_PIN_11
#define ARM2_GPIO_Port GPIOE
#define KEY8_Pin GPIO_PIN_12
#define KEY8_GPIO_Port GPIOE
#define ARM3_Pin GPIO_PIN_13
#define ARM3_GPIO_Port GPIOE
#define ARM4_Pin GPIO_PIN_14
#define ARM4_GPIO_Port GPIOE
#define VALVE_Pin GPIO_PIN_10
#define VALVE_GPIO_Port GPIOB
#define PUMP_Pin GPIO_PIN_11
#define PUMP_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
