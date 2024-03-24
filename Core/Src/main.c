/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "gpdma.h"
#include "gpio.h"
#include "icache.h"
#include "memorymap.h"
#include "sai.h"
#include "tim.h"
#include "ucpd.h"
#include "usart.h"
#include "usb_otg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "arm_math.h"
#include "midi.h"
#include "tusb.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <string.h>

// #define TEST_SWITCH
// #define TEST_INTERRUPTS
// #define TEST_TIMER

#ifdef TEST_INTERRUPTS
const char falling_message[] = "falling interrupts enabled\n";
const char rising_message[] = "rising interrupts enabled\n";
const char enable_it_msg[] = "enabling interrupts\n";
const char disable_it_msg[] = "disabling interrupts\n";

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_BUTTON_Pin) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t *)falling_message,
                      strlen(falling_message), 100);
  }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_BUTTON_Pin) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, (uint8_t *)rising_message,
                      strlen(rising_message), 100);
  }
}
#endif

#ifdef TEST_TIMER
int flag = 0;
uint32_t counter = 0;
#define COUNTER_LIMIT (10U)

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim1) {
    if (counter == COUNTER_LIMIT) {
      counter = 0;
      flag = 1;
    }
    counter++;
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim1) {
    // flag = 1;
  }
}
#endif

bool dma_completed = true;
bool sai_it_completed = true;

#define SAMPLING_FREQUENCY (22500U)
#define CURRENT_NOTE_DURATION (1U) // in s
#define CURRENT_NOTE_LENGTH (SAMPLING_FREQUENCY * CURRENT_NOTE_DURATION)

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
  if (hsai == &hsai_BlockA1) {
    dma_completed = true;
    sai_it_completed = true;
  }
}
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
  if (hsai == &hsai_BlockA1) {
    HAL_UART_Transmit(&huart1, (uint8_t *)"Va Male\r\n", 10, 50);
  }
}

#define SEMITONE_RATIO 1.059463094

/*
  [audio_content, sampl_freq] = audioread("sample_44kH.wav");
  semitone_ration = 1.059463094;
  L = length(audio_content);
  loop_point_44kH = 9591;

  % generare la quinta con signal reconstruction
  % il risultato e' molto buono in realta' considerando che c'e' solamente
  % un signal reconstruction algorithm stupidissimo senza filtri ne niente

  fifth_ratio = semitone_ration ^ 5;
  % test 1: semplice lookup table
  fifth_chord = zeros(1, L);
  for idx = 1:L
      i = (idx * fifth_ratio);
      if i > loop_point_44kH
          i = mod(i, L - loop_point_44kH) + (loop_point_44kH);
      end
      fifth_chord(idx) = audio_content(round(i));
  end

  audiowrite("signal_reconstruction_experiment/fifth.wav", fifth_chord,
  sampl_freq);

  % resampling a 12kHz
  % ok questo fa una cosa diversa
  % resampled = resample(fifth_chord, sampl_freq, 12000);
  % audiowrite("signal_reconstruction_experiment/fifth_resampled.wav",
  resampled, ... % 12000);
*/
void change_semitone(int16_t *current_note, int dsem) {
  double ratio = dsem * SEMITONE_RATIO;
  for (size_t idx = 0; idx < CURRENT_NOTE_LENGTH; idx++) {
    size_t i = (size_t)trunc(idx * ratio);
    if (i > 9591U) {
      i = (i % SAMPLE_44KHZ_SIZE - SAMPLE_44KHZ_LOOP_POINT) +
          (SAMPLE_44KHZ_LOOP_POINT);
    }
    current_note[idx] = sample_44kHz[i];
  }
}

// MIDI USER FUNCTIONS

void board_led_write(bool led_state) {
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, led_state);
}

uint32_t board_millis(void) { return HAL_GetTick(); }

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_UCPD1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_SAI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

#ifdef TEST_TIMER
  const char message[] = "Son vivo\r\n";
#endif

#ifdef TEST_SWITCH
  const char message[] = "Son vivo\r\n";
  uint32_t ptime = HAL_GetTick();
  uint32_t current_time;
  GPIO_PinState pstate = GPIO_PIN_RESET;
#endif

#ifdef TEST_INTERRUPTS
  uint32_t firstts = HAL_GetTick();
#endif

  bool res = tusb_init();
  if (!res) {
    print("TinyUsb initialization failure");
    Error_Handler();
  }

  int16_t current_note[CURRENT_NOTE_LENGTH];
  change_semitone(current_note, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

#if 0
    if (dma_completed) {
      dma_completed = false;
      volatile HAL_StatusTypeDef trasmit_result = HAL_SAI_Transmit_DMA(
          &hsai_BlockA1, (uint16_t *)sample_44kHz, SAMPLE_44KHZ_SIZE);
      HAL_UART_Transmit(&huart1, (uint8_t *)"Vabom\r\n", 8, 50);
    }
#else

    tud_task(); // device task
    led_blinking_task();
    midi_task();

    if (sai_it_completed) {
      sai_it_completed = false;
      // HAL_SAI_Transmit_IT(&hsai_BlockA1, (uint16_t *)current_note,
      // CURRENT_NOTE_LENGTH);

      // HAL_SAI_Transmit_IT(&hsai_BlockA1, (uint16_t*)sample_long_44kHz,
      // SAMPLE_LONG_44KHZ_SIZE);
    }
#endif
    // HAL_SAI_Transmit(&hsai_BlockA1, (uint16_t *)sample_44kHz,
    // SAMPLE_44KHZ_SIZE, 3000);

#ifdef TEST_INTERRUPTS
    while (HAL_GetTick() - firstts < 3000)
      ; // wait 5 seconds

    __disable_irq();
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
                      GPIO_PIN_SET); // notify that interrupts are disabled
    HAL_UART_Transmit(&huart1, (uint8_t *)disable_it_msg,
                      strlen(disable_it_msg), 100);

    GPIO_PinState button_pressed = GPIO_PIN_RESET;
    do {
      button_pressed = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
    } while (button_pressed ==
             GPIO_PIN_RESET); // wait for the button to be pressed

    button_pressed = GPIO_PIN_SET;
    do {
      button_pressed = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
    } while (button_pressed ==
             GPIO_PIN_SET); // wait for the button to be pressed

    button_pressed = GPIO_PIN_RESET;
    do {
      button_pressed = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
    } while (button_pressed ==
             GPIO_PIN_RESET); // wait for the button to be pressed

    button_pressed = GPIO_PIN_SET;
    do {
      button_pressed = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
    } while (button_pressed ==
             GPIO_PIN_SET); // wait for the button to be pressed

    HAL_GPIO_WritePin(
        LED_BLUE_GPIO_Port, LED_BLUE_Pin,
        GPIO_PIN_RESET); // notify that interrupts are enabled again
    HAL_UART_Transmit(&huart1, (uint8_t *)enable_it_msg, strlen(enable_it_msg),
                      100);
    __enable_irq();
    while (1)
      ;
#endif

#ifdef TEST_SWITCH
    GPIO_PinState switch_state =
        HAL_GPIO_ReadPin(USER_SWITCH_GPIO_Port, USER_SWITCH_Pin);
    if (switch_state != pstate) {
      pstate = switch_state;
      HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, switch_state);
    }

    current_time = HAL_GetTick();
    if ((current_time - ptime) > 500) {
      ptime = current_time;
      HAL_UART_Transmit(&huart1, (uint8_t *)message, 10, 100);
      HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }
#endif

#ifdef TEST_TIMER
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
    while (!flag)
      ;
    flag = 0;
    HAL_UART_Transmit(&huart1, (uint8_t *)message, sizeof(message), 100);
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 |
                                     RCC_OSCILLATORTYPE_HSI |
                                     RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
