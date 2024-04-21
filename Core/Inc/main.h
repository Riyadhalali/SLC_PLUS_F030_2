/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define EXIT_Pin GPIO_PIN_3
#define EXIT_GPIO_Port GPIOA
#define DECREMENT_Pin GPIO_PIN_4
#define DECREMENT_GPIO_Port GPIOA
#define INCREMENT_Pin GPIO_PIN_5
#define INCREMENT_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOA
#define Enter_Pin GPIO_PIN_0
#define Enter_GPIO_Port GPIOB
#define Enter_EXTI_IRQn EXTI0_1_IRQn
#define RELAY_L_3_Pin GPIO_PIN_9
#define RELAY_L_3_GPIO_Port GPIOA
#define RELAY_L_2_Pin GPIO_PIN_10
#define RELAY_L_2_GPIO_Port GPIOA
#define RELAY_L_1_Pin GPIO_PIN_11
#define RELAY_L_1_GPIO_Port GPIOA
#define BACKLIGHT_Pin GPIO_PIN_12
#define BACKLIGHT_GPIO_Port GPIOA
#define Ac_Available_Pin GPIO_PIN_7
#define Ac_Available_GPIO_Port GPIOB
#define Ac_Available_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
