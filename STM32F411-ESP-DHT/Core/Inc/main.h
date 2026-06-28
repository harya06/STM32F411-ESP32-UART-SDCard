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
#define INPUT_11_Pin GPIO_PIN_0
#define INPUT_11_GPIO_Port GPIOA
#define SUHU_SENSOR_Pin GPIO_PIN_1
#define SUHU_SENSOR_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define OUTPUT_2_Pin GPIO_PIN_0
#define OUTPUT_2_GPIO_Port GPIOB
#define OUTPUT_1_Pin GPIO_PIN_1
#define OUTPUT_1_GPIO_Port GPIOB
#define OUTPUT_3_Pin GPIO_PIN_12
#define OUTPUT_3_GPIO_Port GPIOB
#define OUTPUT_4_Pin GPIO_PIN_13
#define OUTPUT_4_GPIO_Port GPIOB
#define INPUT_1_Pin GPIO_PIN_14
#define INPUT_1_GPIO_Port GPIOB
#define INPUT_2_Pin GPIO_PIN_15
#define INPUT_2_GPIO_Port GPIOB
#define INPUT_3_Pin GPIO_PIN_8
#define INPUT_3_GPIO_Port GPIOA
#define INPUT_4_Pin GPIO_PIN_11
#define INPUT_4_GPIO_Port GPIOA
#define INPUT_5_Pin GPIO_PIN_12
#define INPUT_5_GPIO_Port GPIOA
#define INPUT_6_Pin GPIO_PIN_15
#define INPUT_6_GPIO_Port GPIOA
#define INPUT_7_Pin GPIO_PIN_4
#define INPUT_7_GPIO_Port GPIOB
#define INPUT_8_Pin GPIO_PIN_5
#define INPUT_8_GPIO_Port GPIOB
#define INPUT_9_Pin GPIO_PIN_8
#define INPUT_9_GPIO_Port GPIOB
#define INPUT_10_Pin GPIO_PIN_9
#define INPUT_10_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
