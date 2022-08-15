/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
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
#define SIM_PWON_Pin GPIO_PIN_13
#define SIM_PWON_GPIO_Port GPIOC
#define SIM_RST_Pin GPIO_PIN_14
#define SIM_RST_GPIO_Port GPIOC
#define CTR_SIM1_Pin GPIO_PIN_15
#define CTR_SIM1_GPIO_Port GPIOC
#define CTR_SIM2_Pin GPIO_PIN_0
#define CTR_SIM2_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define WP_FLH_Pin GPIO_PIN_4
#define WP_FLH_GPIO_Port GPIOC
#define RST_FLH_Pin GPIO_PIN_5
#define RST_FLH_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define INT_ETH_Pin GPIO_PIN_6
#define INT_ETH_GPIO_Port GPIOC
#define INT_ETH_EXTI_IRQn EXTI9_5_IRQn
#define CTR_PWETH_Pin GPIO_PIN_7
#define CTR_PWETH_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOC
#define DE_Pin GPIO_PIN_8
#define DE_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_11
#define IN2_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_12
#define IN1_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_15
#define OUT2_GPIO_Port GPIOA
#define CTR_PW4G_Pin GPIO_PIN_4
#define CTR_PW4G_GPIO_Port GPIOB
#define CTR_GND4G_Pin GPIO_PIN_5
#define CTR_GND4G_GPIO_Port GPIOB
#define NET_Pin GPIO_PIN_8
#define NET_GPIO_Port GPIOB
#define NET_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
