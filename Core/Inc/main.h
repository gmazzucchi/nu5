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
#include "stm32u5xx_hal.h"

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
#define USER_BUTTON_Pin          GPIO_PIN_13
#define USER_BUTTON_GPIO_Port    GPIOC
#define VBUS_SENSE_Pin           GPIO_PIN_2
#define VBUS_SENSE_GPIO_Port     GPIOC
#define LCD_A0_Pin               GPIO_PIN_2
#define LCD_A0_GPIO_Port         GPIOA
#define DAC1_CH1_LEFT_Pin        GPIO_PIN_4
#define DAC1_CH1_LEFT_GPIO_Port  GPIOA
#define DAC1_CH2_RIGHT_Pin       GPIO_PIN_5
#define DAC1_CH2_RIGHT_GPIO_Port GPIOA
#define NOTA2_Pin                GPIO_PIN_0
#define NOTA2_GPIO_Port          GPIOB
#define LCD_CS_Pin               GPIO_PIN_1
#define LCD_CS_GPIO_Port         GPIOB
#define USER_SWITCH_Pin          GPIO_PIN_13
#define USER_SWITCH_GPIO_Port    GPIOF
#define NOTA5_Pin                GPIO_PIN_7
#define NOTA5_GPIO_Port          GPIOE
#define NOTA3_Pin                GPIO_PIN_12
#define NOTA3_GPIO_Port          GPIOE
#define NOTA4_Pin                GPIO_PIN_14
#define NOTA4_GPIO_Port          GPIOE
#define NOTA1_Pin                GPIO_PIN_15
#define NOTA1_GPIO_Port          GPIOE
#define LCD_RST_Pin              GPIO_PIN_10
#define LCD_RST_GPIO_Port        GPIOB
#define LCD1_RS_Pin              GPIO_PIN_11
#define LCD1_RS_GPIO_Port        GPIOB
#define LCD1_D2_Pin              GPIO_PIN_13
#define LCD1_D2_GPIO_Port        GPIOB
#define UCPD_FLT_Pin             GPIO_PIN_14
#define UCPD_FLT_GPIO_Port       GPIOB
#define UCPD1_CC2_Pin            GPIO_PIN_15
#define UCPD1_CC2_GPIO_Port      GPIOB
#define LCD1_D1_Pin              GPIO_PIN_11
#define LCD1_D1_GPIO_Port        GPIOD
#define LCD1_D3_Pin              GPIO_PIN_12
#define LCD1_D3_GPIO_Port        GPIOD
#define LCD1_D6_Pin              GPIO_PIN_14
#define LCD1_D6_GPIO_Port        GPIOD
#define LCD1_D7_Pin              GPIO_PIN_15
#define LCD1_D7_GPIO_Port        GPIOD
#define LED_RED_Pin              GPIO_PIN_2
#define LED_RED_GPIO_Port        GPIOG
#define LCD1_D0_Pin              GPIO_PIN_6
#define LCD1_D0_GPIO_Port        GPIOC
#define LED_GREEN_Pin            GPIO_PIN_7
#define LED_GREEN_GPIO_Port      GPIOC
#define USART1_TX_Pin            GPIO_PIN_9
#define USART1_TX_GPIO_Port      GPIOA
#define USART1_RX_Pin            GPIO_PIN_10
#define USART1_RX_GPIO_Port      GPIOA
#define USB_OTG_FS_DM_Pin        GPIO_PIN_11
#define USB_OTG_FS_DM_GPIO_Port  GPIOA
#define USB_OTG_FS_DP_Pin        GPIO_PIN_12
#define USB_OTG_FS_DP_GPIO_Port  GPIOA
#define UCPD1_CC1_Pin            GPIO_PIN_15
#define UCPD1_CC1_GPIO_Port      GPIOA
#define UCPD_DBn_Pin             GPIO_PIN_5
#define UCPD_DBn_GPIO_Port       GPIOB
#define LED_BLUE_Pin             GPIO_PIN_7
#define LED_BLUE_GPIO_Port       GPIOB
#define LCD1_D4_Pin              GPIO_PIN_8
#define LCD1_D4_GPIO_Port        GPIOB
#define LCD1_D5_Pin              GPIO_PIN_9
#define LCD1_D5_GPIO_Port        GPIOB
#define LCD1_ENABLE_Pin          GPIO_PIN_0
#define LCD1_ENABLE_GPIO_Port    GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
