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
typedef enum {START_ENCODER_MEASURE_DISTANCE=0x0, STOP_ENCODER_MEASURE_DISTANCE,\
    START_ENCODER_CALIBRATION, STOP_ENCODER_CALIBRATION,\
    START_DISTANCE_CALIBRATION, STOP_DISTANCE_CALIBRATION, SAVE_DISTANCE_CALIBRATION, ENCODER_IDLE} encoder_mode;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_MODE2_Pin GPIO_PIN_0
#define ENC_MODE2_GPIO_Port GPIOC
#define ENC_MODE1_Pin GPIO_PIN_1
#define ENC_MODE1_GPIO_Port GPIOC
#define ENC_MODE_INT_Pin GPIO_PIN_0
#define ENC_MODE_INT_GPIO_Port GPIOA
#define ENC_MODE_INT_EXTI_IRQn EXTI0_IRQn
#define ENC_MODE0_Pin GPIO_PIN_1
#define ENC_MODE0_GPIO_Port GPIOA
#define TRG_OUT_Pin GPIO_PIN_0
#define TRG_OUT_GPIO_Port GPIOB
#define ENC_INDEX_INT_Pin GPIO_PIN_7
#define ENC_INDEX_INT_GPIO_Port GPIOC
#define ENC_INDEX_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
