/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define KBD_COL5_Pin GPIO_PIN_13
#define KBD_COL5_GPIO_Port GPIOC
#define RCC_OSC32K_IN_Pin GPIO_PIN_14
#define RCC_OSC32K_IN_GPIO_Port GPIOC
#define RCC_OSC32K_OUT_Pin GPIO_PIN_15
#define RCC_OSC32K_OUT_GPIO_Port GPIOC
#define RCC_OSC12M_IN_Pin GPIO_PIN_0
#define RCC_OSC12M_IN_GPIO_Port GPIOD
#define RCC_OSC12M_OUT_Pin GPIO_PIN_1
#define RCC_OSC12M_OUT_GPIO_Port GPIOD
#define LCD_D0_Pin GPIO_PIN_0
#define LCD_D0_GPIO_Port GPIOA
#define LCD_D1_Pin GPIO_PIN_1
#define LCD_D1_GPIO_Port GPIOA
#define LCD_D2_Pin GPIO_PIN_2
#define LCD_D2_GPIO_Port GPIOA
#define LCD_D3_Pin GPIO_PIN_3
#define LCD_D3_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_1
#define LCD_CS_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_2
#define LCD_RS_GPIO_Port GPIOB
#define LCD_WR_Pin GPIO_PIN_10
#define LCD_WR_GPIO_Port GPIOB
#define LATCH_SS_Pin GPIO_PIN_11
#define LATCH_SS_GPIO_Port GPIOB
#define HY3131_SS_Pin GPIO_PIN_12
#define HY3131_SS_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define BACKLIGHT_Pin GPIO_PIN_13
#define BACKLIGHT_GPIO_Port GPIOA
#define KBD_COL4_Pin GPIO_PIN_14
#define KBD_COL4_GPIO_Port GPIOA
#define KBD_COL3_Pin GPIO_PIN_15
#define KBD_COL3_GPIO_Port GPIOA
#define KBD_COL2_Pin GPIO_PIN_3
#define KBD_COL2_GPIO_Port GPIOB
#define KBD_COL1_Pin GPIO_PIN_4
#define KBD_COL1_GPIO_Port GPIOB
#define KBD_ROW1_Pin GPIO_PIN_5
#define KBD_ROW1_GPIO_Port GPIOB
#define KBD_ROW2_Pin GPIO_PIN_6
#define KBD_ROW2_GPIO_Port GPIOB
#define KBD_ROW3_Pin GPIO_PIN_7
#define KBD_ROW3_GPIO_Port GPIOB
#define KBD_ROW4_Pin GPIO_PIN_8
#define KBD_ROW4_GPIO_Port GPIOB
#define KBD_ROW5_Pin GPIO_PIN_9
#define KBD_ROW5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
