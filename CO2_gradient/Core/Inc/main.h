/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32wlxx_hal.h"

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
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define MUX_A1_Pin GPIO_PIN_3
#define MUX_A1_GPIO_Port GPIOB
#define MUX_A2_Pin GPIO_PIN_4
#define MUX_A2_GPIO_Port GPIOB
#define SOILTH_ONOFF_Pin GPIO_PIN_0
#define SOILTH_ONOFF_GPIO_Port GPIOA
#define CS_SD_Pin GPIO_PIN_4
#define CS_SD_GPIO_Port GPIOA
#define MOS_SD_ONOFF_Pin GPIO_PIN_5
#define MOS_SD_ONOFF_GPIO_Port GPIOA
#define MUXCO2_EN_Pin GPIO_PIN_8
#define MUXCO2_EN_GPIO_Port GPIOA
#define MUX_A0_Pin GPIO_PIN_9
#define MUX_A0_GPIO_Port GPIOA
#define MUXADC_EN_Pin GPIO_PIN_11
#define MUXADC_EN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
