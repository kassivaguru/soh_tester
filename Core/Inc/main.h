/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f030x6.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_SW_1_Pin GPIO_PIN_0
#define USER_SW_1_GPIO_Port GPIOF
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOF
#define RS485_EN_Pin GPIO_PIN_0
#define RS485_EN_GPIO_Port GPIOB
#define PRECHG_EN_Pin GPIO_PIN_11
#define PRECHG_EN_GPIO_Port GPIOA
#define INPUT_EN_Pin GPIO_PIN_12
#define INPUT_EN_GPIO_Port GPIOA
#define CELL_EN_Pin GPIO_PIN_15
#define CELL_EN_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_3
#define LED_GREEN_GPIO_Port GPIOB
#define LED_DEBUG_Pin GPIO_PIN_4
#define LED_DEBUG_GPIO_Port GPIOB
#define DCH_OVRD_Pin GPIO_PIN_6
#define DCH_OVRD_GPIO_Port GPIOB
#define BLEED_EN_Pin GPIO_PIN_7
#define BLEED_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
