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
#include "stm32f7xx_hal.h"

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
#define GPIO_USER_SW_Pin GPIO_PIN_13
#define GPIO_USER_SW_GPIO_Port GPIOC
#define D0A_Pin GPIO_PIN_0
#define D0A_GPIO_Port GPIOF
#define D1A_Pin GPIO_PIN_1
#define D1A_GPIO_Port GPIOF
#define D2A_Pin GPIO_PIN_2
#define D2A_GPIO_Port GPIOF
#define D3A_Pin GPIO_PIN_3
#define D3A_GPIO_Port GPIOF
#define D4A_Pin GPIO_PIN_4
#define D4A_GPIO_Port GPIOF
#define D5A_Pin GPIO_PIN_5
#define D5A_GPIO_Port GPIOF
#define D6A_Pin GPIO_PIN_6
#define D6A_GPIO_Port GPIOF
#define D7A_Pin GPIO_PIN_7
#define D7A_GPIO_Port GPIOF
#define D8A_Pin GPIO_PIN_8
#define D8A_GPIO_Port GPIOF
#define D9A_Pin GPIO_PIN_9
#define D9A_GPIO_Port GPIOF
#define CT0_Pin GPIO_PIN_1
#define CT0_GPIO_Port GPIOB
#define CT1_Pin GPIO_PIN_2
#define CT1_GPIO_Port GPIOB
#define CT2_Pin GPIO_PIN_11
#define CT2_GPIO_Port GPIOF
#define CT3_Pin GPIO_PIN_12
#define CT3_GPIO_Port GPIOF
#define CT4_Pin GPIO_PIN_13
#define CT4_GPIO_Port GPIOF
#define CT5_Pin GPIO_PIN_14
#define CT5_GPIO_Port GPIOF
#define CT6_Pin GPIO_PIN_15
#define CT6_GPIO_Port GPIOF
#define CT7_Pin GPIO_PIN_0
#define CT7_GPIO_Port GPIOG
#define CT8_Pin GPIO_PIN_1
#define CT8_GPIO_Port GPIOG
#define CT9_Pin GPIO_PIN_7
#define CT9_GPIO_Port GPIOE
#define CT10_Pin GPIO_PIN_8
#define CT10_GPIO_Port GPIOE
#define CT11_Pin GPIO_PIN_9
#define CT11_GPIO_Port GPIOE
#define CT12_Pin GPIO_PIN_10
#define CT12_GPIO_Port GPIOE
#define CT13_Pin GPIO_PIN_11
#define CT13_GPIO_Port GPIOE
#define CT14_Pin GPIO_PIN_12
#define CT14_GPIO_Port GPIOE
#define CT15_Pin GPIO_PIN_13
#define CT15_GPIO_Port GPIOE
#define CT16_Pin GPIO_PIN_14
#define CT16_GPIO_Port GPIOE
#define CT17_Pin GPIO_PIN_15
#define CT17_GPIO_Port GPIOE
#define CT18_Pin GPIO_PIN_10
#define CT18_GPIO_Port GPIOB
#define CT19_Pin GPIO_PIN_11
#define CT19_GPIO_Port GPIOB
#define USER_LED_GREEN_Pin GPIO_PIN_14
#define USER_LED_GREEN_GPIO_Port GPIOB
#define CT23_Pin GPIO_PIN_2
#define CT23_GPIO_Port GPIOG
#define CT22_Pin GPIO_PIN_3
#define CT22_GPIO_Port GPIOG
#define CT21_Pin GPIO_PIN_4
#define CT21_GPIO_Port GPIOG
#define CT20_Pin GPIO_PIN_5
#define CT20_GPIO_Port GPIOG
#define D0B_Pin GPIO_PIN_6
#define D0B_GPIO_Port GPIOG
#define D1B_Pin GPIO_PIN_7
#define D1B_GPIO_Port GPIOG
#define D2B_Pin GPIO_PIN_8
#define D2B_GPIO_Port GPIOG
#define D3B_Pin GPIO_PIN_9
#define D3B_GPIO_Port GPIOG
#define D4B_Pin GPIO_PIN_10
#define D4B_GPIO_Port GPIOG
#define D5B_Pin GPIO_PIN_11
#define D5B_GPIO_Port GPIOG
#define D6B_Pin GPIO_PIN_12
#define D6B_GPIO_Port GPIOG
#define D7B_Pin GPIO_PIN_13
#define D7B_GPIO_Port GPIOG
#define D8B_Pin GPIO_PIN_14
#define D8B_GPIO_Port GPIOG
#define D9B_Pin GPIO_PIN_15
#define D9B_GPIO_Port GPIOG
#define USER_LED_BLUE_Pin GPIO_PIN_7
#define USER_LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
