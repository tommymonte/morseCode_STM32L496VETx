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
#include "stm32l4xx_hal.h"

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
#define IO2_Pin GPIO_PIN_2
#define IO2_GPIO_Port GPIOE
#define IO3_Pin GPIO_PIN_3
#define IO3_GPIO_Port GPIOE
#define IO4_Pin GPIO_PIN_4
#define IO4_GPIO_Port GPIOE
#define IO5_Pin GPIO_PIN_5
#define IO5_GPIO_Port GPIOE
#define IO6_Pin GPIO_PIN_6
#define IO6_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOC
#define AIN3_Pin GPIO_PIN_2
#define AIN3_GPIO_Port GPIOC
#define AIN4_Pin GPIO_PIN_3
#define AIN4_GPIO_Port GPIOC
#define OA1_INP_Pin GPIO_PIN_0
#define OA1_INP_GPIO_Port GPIOA
#define OA1_INM_Pin GPIO_PIN_1
#define OA1_INM_GPIO_Port GPIOA
#define AIN5_Pin GPIO_PIN_2
#define AIN5_GPIO_Port GPIOA
#define OA1_OUT_Pin GPIO_PIN_3
#define OA1_OUT_GPIO_Port GPIOA
#define DAC_OUT1_Pin GPIO_PIN_4
#define DAC_OUT1_GPIO_Port GPIOA
#define DAC_OUT2_Pin GPIO_PIN_5
#define DAC_OUT2_GPIO_Port GPIOA
#define OA1_INPA6_Pin GPIO_PIN_6
#define OA1_INPA6_GPIO_Port GPIOA
#define OA2_INM_Pin GPIO_PIN_7
#define OA2_INM_GPIO_Port GPIOA
#define COMP_INM_Pin GPIO_PIN_4
#define COMP_INM_GPIO_Port GPIOC
#define COMP_INP_Pin GPIO_PIN_5
#define COMP_INP_GPIO_Port GPIOC
#define OA2_COMP_OUT_Pin GPIO_PIN_0
#define OA2_COMP_OUT_GPIO_Port GPIOB
#define VLCD_Pin GPIO_PIN_2
#define VLCD_GPIO_Port GPIOB
#define IO7_Pin GPIO_PIN_7
#define IO7_GPIO_Port GPIOE
#define IO8_Pin GPIO_PIN_8
#define IO8_GPIO_Port GPIOE
#define IO9_Pin GPIO_PIN_9
#define IO9_GPIO_Port GPIOE
#define IO10_Pin GPIO_PIN_10
#define IO10_GPIO_Port GPIOE
#define IO11_Pin GPIO_PIN_11
#define IO11_GPIO_Port GPIOE
#define IO12_Pin GPIO_PIN_12
#define IO12_GPIO_Port GPIOE
#define IO13_Pin GPIO_PIN_13
#define IO13_GPIO_Port GPIOE
#define IO14_Pin GPIO_PIN_14
#define IO14_GPIO_Port GPIOE
#define IO15_Pin GPIO_PIN_15
#define IO15_GPIO_Port GPIOE
#define SW6_Pin GPIO_PIN_10
#define SW6_GPIO_Port GPIOB
#define SW7_Pin GPIO_PIN_11
#define SW7_GPIO_Port GPIOB
#define SW7_EXTI_IRQn EXTI15_10_IRQn
#define IO24_Pin GPIO_PIN_8
#define IO24_GPIO_Port GPIOD
#define IO25_Pin GPIO_PIN_9
#define IO25_GPIO_Port GPIOD
#define IO26_Pin GPIO_PIN_10
#define IO26_GPIO_Port GPIOD
#define IO27_Pin GPIO_PIN_11
#define IO27_GPIO_Port GPIOD
#define IO28_Pin GPIO_PIN_12
#define IO28_GPIO_Port GPIOD
#define IO29_Pin GPIO_PIN_13
#define IO29_GPIO_Port GPIOD
#define IO30_Pin GPIO_PIN_14
#define IO30_GPIO_Port GPIOD
#define IO31_Pin GPIO_PIN_15
#define IO31_GPIO_Port GPIOD
#define USB_DN_Pin GPIO_PIN_11
#define USB_DN_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_10
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOC
#define IO16_Pin GPIO_PIN_0
#define IO16_GPIO_Port GPIOD
#define IO17_Pin GPIO_PIN_1
#define IO17_GPIO_Port GPIOD
#define IO18_Pin GPIO_PIN_2
#define IO18_GPIO_Port GPIOD
#define IO19_Pin GPIO_PIN_3
#define IO19_GPIO_Port GPIOD
#define IO20_Pin GPIO_PIN_4
#define IO20_GPIO_Port GPIOD
#define IO21_Pin GPIO_PIN_5
#define IO21_GPIO_Port GPIOD
#define IO22_Pin GPIO_PIN_6
#define IO22_GPIO_Port GPIOD
#define IO23_Pin GPIO_PIN_7
#define IO23_GPIO_Port GPIOD
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define SW0_EXTI_IRQn EXTI3_IRQn
#define SW1_Pin GPIO_PIN_4
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_5
#define SW2_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_6
#define SW3_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_7
#define SW4_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_8
#define SW5_GPIO_Port GPIOB
#define IO0_Pin GPIO_PIN_0
#define IO0_GPIO_Port GPIOE
#define IO1_Pin GPIO_PIN_1
#define IO1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
