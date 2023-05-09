/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define GPIO_Output_EV_P_Pin GPIO_PIN_0
#define GPIO_Output_EV_P_GPIO_Port GPIOF
#define GPIO_Input_AU_STATUS_Pin GPIO_PIN_1
#define GPIO_Input_AU_STATUS_GPIO_Port GPIOF
#define ADC1_IN1_A_TEMP_Pin GPIO_PIN_0
#define ADC1_IN1_A_TEMP_GPIO_Port GPIOA
#define ADC1_IN2_A_PRES_Pin GPIO_PIN_1
#define ADC1_IN2_A_PRES_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define GPIO_Output_EV_3_Pin GPIO_PIN_3
#define GPIO_Output_EV_3_GPIO_Port GPIOA
#define GPIO_Output_EV_2_Pin GPIO_PIN_4
#define GPIO_Output_EV_2_GPIO_Port GPIOA
#define TIM2_CH1_DS_COMP_Pin GPIO_PIN_5
#define TIM2_CH1_DS_COMP_GPIO_Port GPIOA
#define TIM3_CH1_DS_TURB_Pin GPIO_PIN_6
#define TIM3_CH1_DS_TURB_GPIO_Port GPIOA
#define GPIO_Output_EV_1_Pin GPIO_PIN_7
#define GPIO_Output_EV_1_GPIO_Port GPIOA
#define TIM3_CH4_DS_M2_Pin GPIO_PIN_1
#define TIM3_CH4_DS_M2_GPIO_Port GPIOB
#define TIM1_CH1_DS_M3_Pin GPIO_PIN_8
#define TIM1_CH1_DS_M3_GPIO_Port GPIOA
#define TIM2_CH3_DS_M1_Pin GPIO_PIN_9
#define TIM2_CH3_DS_M1_GPIO_Port GPIOA
#define CAN_RD_Pin GPIO_PIN_11
#define CAN_RD_GPIO_Port GPIOA
#define CAN_TD_Pin GPIO_PIN_12
#define CAN_TD_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_15
#define USART2_RX_GPIO_Port GPIOA
#define Built_in_LED_Pin GPIO_PIN_3
#define Built_in_LED_GPIO_Port GPIOB
#define GPIO_Output_LED_EN_Pin GPIO_PIN_4
#define GPIO_Output_LED_EN_GPIO_Port GPIOB
#define GPIO_Output_LCD_EN_Pin GPIO_PIN_5
#define GPIO_Output_LCD_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
