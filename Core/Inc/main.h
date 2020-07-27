/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
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
#define RGB_DATA_Pin GPIO_PIN_0
#define RGB_DATA_GPIO_Port GPIOA
#define LIGHT_SENSOR_Pin GPIO_PIN_1
#define LIGHT_SENSOR_GPIO_Port GPIOA
#define LED_PWM_Pin GPIO_PIN_3
#define LED_PWM_GPIO_Port GPIOA
#define LED_DATA_LATCH_Pin GPIO_PIN_4
#define LED_DATA_LATCH_GPIO_Port GPIOA
#define USART3_DE_Pin GPIO_PIN_2
#define USART3_DE_GPIO_Port GPIOB
#define BTN0_Pin GPIO_PIN_12
#define BTN0_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_13
#define BTN1_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_14
#define BTN2_GPIO_Port GPIOB
#define BTN3_Pin GPIO_PIN_15
#define BTN3_GPIO_Port GPIOB
#define BTN4_Pin GPIO_PIN_6
#define BTN4_GPIO_Port GPIOC
#define BTN5_Pin GPIO_PIN_7
#define BTN5_GPIO_Port GPIOC
#define BTN6_Pin GPIO_PIN_8
#define BTN6_GPIO_Port GPIOC
#define BTN7_Pin GPIO_PIN_9
#define BTN7_GPIO_Port GPIOC
#define BUZZ_Pin GPIO_PIN_9
#define BUZZ_GPIO_Port GPIOA
#define BMP280_NSS_Pin GPIO_PIN_15
#define BMP280_NSS_GPIO_Port GPIOA
#define BMP280_SCK_Pin GPIO_PIN_10
#define BMP280_SCK_GPIO_Port GPIOC
#define BMP280_MISO_Pin GPIO_PIN_11
#define BMP280_MISO_GPIO_Port GPIOC
#define BMP280_MOSI_Pin GPIO_PIN_12
#define BMP280_MOSI_GPIO_Port GPIOC
#define DS3231_N_INT_Pin GPIO_PIN_4
#define DS3231_N_INT_GPIO_Port GPIOB
#define DS3231_N_RES_Pin GPIO_PIN_5
#define DS3231_N_RES_GPIO_Port GPIOB
#define SRV_BTN_Pin GPIO_PIN_8
#define SRV_BTN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
