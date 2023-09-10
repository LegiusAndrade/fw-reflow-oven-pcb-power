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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx.h"

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
#define LED_COMM_Pin GPIO_PIN_13
#define LED_COMM_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_14
#define LED_STATUS_GPIO_Port GPIOC
#define VREG_Pin GPIO_PIN_0
#define VREG_GPIO_Port GPIOA
#define USART_RX_TX_Pin GPIO_PIN_1
#define USART_RX_TX_GPIO_Port GPIOA
#define CS_MAX31855_Pin GPIO_PIN_4
#define CS_MAX31855_GPIO_Port GPIOA
#define IOUT_Pin GPIO_PIN_7
#define IOUT_GPIO_Port GPIOA
#define NTC_Pin GPIO_PIN_0
#define NTC_GPIO_Port GPIOB
#define LIN_Pin GPIO_PIN_1
#define LIN_GPIO_Port GPIOB
#define PD_Pin GPIO_PIN_2
#define PD_GPIO_Port GPIOB
#define RELAY_INRUSH_Pin GPIO_PIN_11
#define RELAY_INRUSH_GPIO_Port GPIOB
#define PG_MPQ_Pin GPIO_PIN_12
#define PG_MPQ_GPIO_Port GPIOB
#define PG_MPQ_EXTI_IRQn EXTI15_10_IRQn
#define FAN2_RPM_Pin GPIO_PIN_13
#define FAN2_RPM_GPIO_Port GPIOB
#define FAN2_OUT_Pin GPIO_PIN_14
#define FAN2_OUT_GPIO_Port GPIOB
#define VBUS_Pin GPIO_PIN_15
#define VBUS_GPIO_Port GPIOB
#define FAN1_RPM_Pin GPIO_PIN_8
#define FAN1_RPM_GPIO_Port GPIOA
#define FAN1_OUT_Pin GPIO_PIN_9
#define FAN1_OUT_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOA
#define FANPCB_RPM_Pin GPIO_PIN_12
#define FANPCB_RPM_GPIO_Port GPIOA
#define FAULT_SD_Pin GPIO_PIN_5
#define FAULT_SD_GPIO_Port GPIOB
#define FAULT_SD_EXTI_IRQn EXTI9_5_IRQn
#define SY_FLT_Pin GPIO_PIN_6
#define SY_FLT_GPIO_Port GPIOB
#define FAULT_CLR_Pin GPIO_PIN_7
#define FAULT_CLR_GPIO_Port GPIOB
#define HIN_Pin GPIO_PIN_9
#define HIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
