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
#include "dac.h"
#include "gpdma.h"
#include "gpio.h"
#include "icache.h"
#include "memorymap.h"
#include "sai.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "all_samples.h"
#include "arm_math.h"
#include "math.h"
#include "pedalinator_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifdef PEDALINATOR_COM_VIRTUAL_PORT_EXAMPLE
#include "tinyusb_pedalinator_porting.h"

#include <tusb.h>
#endif

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
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <string.h>

#ifdef TEST_INTERRUPTS
const char falling_message[] = "falling interrupts enabled\n";
const char rising_message[]  = "rising interrupts enabled\n";
const char enable_it_msg[]   = "enabling interrupts\n";
const char disable_it_msg[]  = "disabling interrupts\n";

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == USER_BUTTON_Pin) {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_UART_Transmit(&huart1, (uint8_t *)falling_message, strlen(falling_message), 100);
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == USER_BUTTON_Pin) {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, (uint8_t *)rising_message, strlen(rising_message), 100);
    }
}
#endif

#ifdef TEST_TIMER
int flag         = 0;
uint32_t counter = 0;
#define COUNTER_LIMIT (10U)

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) {
        if (counter == COUNTER_LIMIT) {
            counter = 0;
            flag    = 1;
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

// #define SAMPLING_FREQUENCY (22500U)
// #define CURRENT_NOTE_DURATION (1U) // in s
// #define CURRENT_NOTE_LENGTH (SAMPLING_FREQUENCY * CURRENT_NOTE_DURATION)

#ifdef PEDALINATOR_HSAI_IT

bool dma_completed    = true;
bool sai_it_completed = true;

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai == &hsai_BlockA1) {
        dma_completed    = true;
        sai_it_completed = true;
    }
}
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
    if (hsai == &hsai_BlockA1) {
        while (1) {
            HAL_UART_Transmit(&huart1, (uint8_t *)"SAI peripheral ERROR\r\n", 23, 50);
        }
    }
}
#endif

const char falling_message[] = "falling interrupts enabled\r\n";
const char rising_message[]  = "rising interrupts enabled\r\n";
const char enable_it_msg[]   = "enabling interrupts\r\n";
const char disable_it_msg[]  = "disabling interrupts\r\n";

bool semitone_up = false;

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == USER_BUTTON_Pin) {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_UART_Transmit(&huart1, (uint8_t *)falling_message, sizeof(falling_message), 100);
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == USER_BUTTON_Pin) {
        semitone_up = true;
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, (uint8_t *)rising_message, sizeof(rising_message), 100);
    }
}

#ifdef PEDALINATOR_PITCH_MODULATION

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

/***
 * This version works well with 1 semitone increase but already with 2 semitone
 * increase it gives distortion.
 * TODO: Idea. Divide the sample in attacco and corpo and then try to prolong
 * the corpo only.
 *
 * IMPORTANT: Technically the loop point has both a start and a finish, so make
 * sure that both of them are present!!!! Here only the starting point is
 * considered and the ending point is used
 */

int change_semitone(
    int16_t *current_note,
    size_t current_note_len,
    int16_t *current_note_old,
    size_t current_note_old_len,
    uint8_t dsem,
    size_t attacco_len,
    size_t corpo_len,
    size_t decay_len,
    size_t loop_point) {
    // powl();
    double ratio = pow(2.0, (dsem / 12.0));
    // *current_note_old_len = (size_t)(((long double)*current_note_old_len) /
    // ratio);
    size_t c_current_note_len = (size_t)(((long double)current_note_len) / ratio);
    size_t c_attacco_len      = (size_t)(((long double)attacco_len) / ratio);
    size_t c_corpo_len        = (size_t)(((long double)corpo_len) / ratio);
    size_t c_decay_len        = (size_t)(((long double)decay_len) / ratio);
    // size_t c_loop_point = (size_t)(((long double)loop_point) / ratio);
    size_t c_current_note_old_len = current_note_old_len;

    // size_t c_current_note_len = *current_note_len;
    // size_t c_attacco_len = *attacco_len;
    // size_t c_corpo_len = *corpo_len;
    // size_t c_decay_len = *decay_len;
    // size_t c_loop_point = *loop_point;
    if (c_current_note_len > MAX_CURRENT_NOTE_L || c_attacco_len > MAX_ATTACCO_L || c_corpo_len > MAX_CORPO_L ||
        c_decay_len > MAX_DECAY_L) {
        return -1;
    }
    if (c_current_note_len > c_current_note_old_len)
        return -1;

    /*
      IL = len(corpo_frames)
      L = math.trunc(len(corpo_frames) / ratio)
      corpo_frames_2 = [0] * L
      for i in range(0, L):
          x = (i * ratio)
          y = x - math.trunc(x)
          z = math.trunc(x) % IL
          corpo_frames_2[i] = corpo_frames[z] * \
              (1 - y) + corpo_frames[(z + 1) % IL] * y
  */
    for (size_t iattacco = 0; iattacco < c_attacco_len; iattacco++) {
        double x               = (iattacco * ratio);
        double y               = x - (int)x;
        size_t z               = (size_t)x % (c_attacco_len);
        current_note[iattacco] = current_note_old[z] * (1 - y) + current_note_old[(z + 1) % c_attacco_len] * y;
    }

    for (size_t icorpo = 0; icorpo < c_corpo_len; icorpo++) {
        double x                             = (icorpo * ratio);
        double y                             = x - (int)x;
        size_t z                             = (size_t)x % (c_corpo_len);
        current_note[icorpo + c_attacco_len] = current_note_old[z + c_attacco_len] * (1 - y) +
                                               current_note_old[((z + 1) % c_corpo_len) + c_attacco_len] * y;
    }

    /* // The original one
  for (size_t idx = 0; idx < c_current_note_len; idx++) {
    double x = (idx * ratio);
    double y = x - (int)x;
    size_t z = (size_t)x % (c_current_note_len);
    current_note[idx] = current_note_old[z] * (1 - y) + current_note_old[(z + 1)
  % c_current_note_len] * y;
  }
   */
    // *current_note_old_len = c_current_note_len;
    return c_current_note_len;
}

/* int ottava_sopra(int16_t *curr_n, size_t curr_n_size, int16_t *base_n,
                 size_t base_n_size, int dsem) {
  if (curr_n_size > base_n_size)
    return -1;
  for (size_t idx = 0; idx < curr_n_size; idx++) {
    curr_n[idx] = base_n[(idx * 2) % curr_n_size];
  }
  return curr_n_size;
} */

int elaborate_note_components(
    int16_t *current_note,
    size_t current_note_len,
    int16_t *attacco,
    size_t attacco_l,
    int16_t *corpo,
    size_t corpo_l,
    int16_t *decay,
    size_t decay_l) {
    double TAU = (((double)corpo_l) / 15.0);
    for (size_t idx = 0; idx < current_note_len; idx++) {
        int16_t val = current_note[idx];
        if (idx < attacco_l) {
            attacco[idx] = val;
        } else {
            size_t jidx               = idx - attacco_l;
            corpo[jidx]               = val;
            corpo[(decay_l) + (jidx)] = val;
            decay[jidx]               = val * exp(-(jidx / TAU));
        }
    }
    return 0;
}

/*
int elaborate_decay(int16_t *decay_n, size_t decay_n_size, int16_t *base_n,
size_t base_n_size) { if (decay_n_size >= base_n_size) { return -1;
  }
  for (size_t jidx = 0; jidx < decay_n_size; jidx++) {
    decay_n[jidx] = base_n[jidx] * exp(-(jidx / TAU));
  }
  return decay_n_size;
}
 */

void build_chord(int16_t *base_note, uint8_t *semitones, size_t n_notes) {
}
#endif

#ifdef PEDALINATOR_ALL_NOTES_STORED
/***
 * TODO: setup a basic system to play all notes
 * check if the ram is enough
 */

size_t attacchi_l[PEDALINATOR_N_NOTES] = {
    // SAMPLE_C1_22KHZ_ATTACCO_L,       SAMPLE_C_SHARP1_22KHZ_ATTACCO_L,
    // SAMPLE_D1_22KHZ_ATTACCO_L,       SAMPLE_EB1_22KHZ_ATTACCO_L,
    SAMPLE_E1_22KHZ_ATTACCO_L,
    SAMPLE_F1_22KHZ_ATTACCO_L,
    SAMPLE_F_SHARP1_22KHZ_ATTACCO_L,
    SAMPLE_G1_22KHZ_ATTACCO_L,
    SAMPLE_G_SHARP1_22KHZ_ATTACCO_L,
    SAMPLE_A1_22KHZ_ATTACCO_L,
    SAMPLE_BB1_22KHZ_ATTACCO_L,
    SAMPLE_B1_22KHZ_ATTACCO_L,
    /* SAMPLE_C2_22KHZ_ATTACCO_L,       SAMPLE_C_SHARP2_22KHZ_ATTACCO_L,
    SAMPLE_D2_22KHZ_ATTACCO_L,       SAMPLE_EB2_22KHZ_ATTACCO_L,
    SAMPLE_E2_22KHZ_ATTACCO_L,       SAMPLE_F2_22KHZ_ATTACCO_L,
    SAMPLE_F_SHARP2_22KHZ_ATTACCO_L, SAMPLE_G2_22KHZ_ATTACCO_L,
    SAMPLE_G_SHARP2_22KHZ_ATTACCO_L, SAMPLE_A2_22KHZ_ATTACCO_L,
    SAMPLE_BB2_22KHZ_ATTACCO_L,      SAMPLE_B2_22KHZ_ATTACCO_L,
    SAMPLE_C3_22KHZ_ATTACCO_L */
};

size_t corpi_l[PEDALINATOR_N_NOTES] = {
    // SAMPLE_C1_22KHZ_CORPO_L,       SAMPLE_C_SHARP1_22KHZ_CORPO_L,
    // SAMPLE_D1_22KHZ_CORPO_L,       SAMPLE_EB1_22KHZ_CORPO_L,
    SAMPLE_E1_22KHZ_CORPO_L,
    SAMPLE_F1_22KHZ_CORPO_L,
    SAMPLE_F_SHARP1_22KHZ_CORPO_L,
    SAMPLE_G1_22KHZ_CORPO_L,
    SAMPLE_G_SHARP1_22KHZ_CORPO_L,
    SAMPLE_A1_22KHZ_CORPO_L,
    SAMPLE_BB1_22KHZ_CORPO_L,
    SAMPLE_B1_22KHZ_CORPO_L,

    /* SAMPLE_C2_22KHZ_CORPO_L,       SAMPLE_C_SHARP2_22KHZ_CORPO_L,
    SAMPLE_D2_22KHZ_CORPO_L,       SAMPLE_EB2_22KHZ_CORPO_L,
    SAMPLE_E2_22KHZ_CORPO_L,       SAMPLE_F2_22KHZ_CORPO_L,
    SAMPLE_F_SHARP2_22KHZ_CORPO_L, SAMPLE_G2_22KHZ_CORPO_L,
    SAMPLE_G_SHARP2_22KHZ_CORPO_L, SAMPLE_A2_22KHZ_CORPO_L,
    SAMPLE_BB2_22KHZ_CORPO_L,      SAMPLE_B2_22KHZ_CORPO_L,
    SAMPLE_C3_22KHZ_CORPO_L */
};

int16_t *attacchi[PEDALINATOR_N_NOTES] = {
    /* sample_C1_22kHz_attacco,       sample_C_sharp1_22kHz_attacco,
    sample_D1_22kHz_attacco,       sample_Eb1_22kHz_attacco, */
    sample_E1_22kHz_attacco,
    sample_F1_22kHz_attacco,
    sample_F_sharp1_22kHz_attacco,
    sample_G1_22kHz_attacco,
    sample_G_sharp1_22kHz_attacco,
    sample_A1_22kHz_attacco,
    sample_Bb1_22kHz_attacco,
    sample_B1_22kHz_attacco,
    /* sample_C2_22kHz_attacco,       sample_C_sharp2_22kHz_attacco,
    sample_D2_22kHz_attacco,       sample_Eb2_22kHz_attacco,
    sample_E2_22kHz_attacco,       sample_F2_22kHz_attacco,
    sample_F_sharp2_22kHz_attacco, sample_G2_22kHz_attacco,
    sample_G_sharp2_22kHz_attacco, sample_A2_22kHz_attacco,
    sample_Bb2_22kHz_attacco,      sample_B2_22kHz_attacco,
    sample_C3_22kHz_attacco */
};

int16_t *corpi[PEDALINATOR_N_NOTES] = {
    // sample_C1_22kHz_corpo,       sample_C_sharp1_22kHz_corpo,
    // sample_D1_22kHz_corpo,       sample_Eb1_22kHz_corpo,
    sample_E1_22kHz_corpo,
    sample_F1_22kHz_corpo,
    sample_F_sharp1_22kHz_corpo,
    sample_G1_22kHz_corpo,
    sample_G_sharp1_22kHz_corpo,
    sample_A1_22kHz_corpo,
    sample_Bb1_22kHz_corpo,
    sample_B1_22kHz_corpo,
    /* sample_C2_22kHz_corpo,       sample_C_sharp2_22kHz_corpo,
    sample_D2_22kHz_corpo,       sample_Eb2_22kHz_corpo,
    sample_E2_22kHz_corpo,       sample_F2_22kHz_corpo,
    sample_F_sharp2_22kHz_corpo, sample_G2_22kHz_corpo,
    sample_G_sharp2_22kHz_corpo, sample_A2_22kHz_corpo,
    sample_Bb2_22kHz_corpo,      sample_B2_22kHz_corpo,
    sample_C3_22kHz_corpo */
};

void elaborate_decay(int16_t *corpo, size_t corpo_len, int16_t *decay, size_t decay_len) {
    double TAU = (((double)corpo_len) / 15.0);
    for (size_t idx = 0; idx < decay_len; idx++) {
        int16_t val = corpo[idx];
        decay[idx]  = val * exp(-(idx / TAU));
    }
}

#endif

#ifdef PEDALINATOR_ADVANCED_PITCH_MODULATION
#include "sample_22kHz_D2.h"

/* // WORKING PYTHON ALGORITHM
 lp = LOOP_POINT_DATA[sample]["lp_start"]
ls = LOOP_POINT_DATA[sample]["lp_stop"]
(frames, nframes) = soundfile.read(sample + ".wav")
attacco_frames = frames[:lp]
corpo_frames = frames[lp:ls]
dsem = 4
ratio = 2**(dsem/12)
IL = len(corpo_frames)
L = math.trunc(len(corpo_frames) / ratio)
corpo_frames_2 = [0] * L
for i in range(0, L):
    x = (i * ratio)
    y = x - math.trunc(x)
    z = math.trunc(x) % IL
    corpo_frames_2[i] = corpo_frames[z] * \
        (1 - y) + corpo_frames[(z + 1) % IL] * y */

int attacco_pitch_shifting(int16_t *curr, size_t curr_len, int16_t *base, size_t base_len, int dsem) {
    if (dsem == 0) {
        return 0;
    }
    double ratio = pow(2.0, dsem / 12.0);
    size_t L     = base_len / ratio;
    size_t IL    = base_len;
    for (size_t i = 0; i < L; i++) {
        double x = i * ratio;
        size_t y = (size_t)x;
        size_t z = (size_t)x % IL;
        curr[i]  = base[z] * (1 - y) + base[(z + 1) % IL] * y;
    }
    return 0;
}

// returns the new current note length
size_t corpo_pitch_shifting(int16_t *curr, size_t curr_max_len, int16_t *base, size_t base_len, int dsem) {
    if (dsem == 0) {
        return 0;
    }
    long double ratio = powl(2.0, dsem / 12.0);
    size_t L          = (size_t)((double)base_len / ratio);
    if (curr_max_len < L) {
        Error_Handler();
    }
    for (size_t i = 0; i < L; i++) {
        double x = (double)i * ratio;
        double y = x - (size_t)x;
        size_t z = (size_t)x % base_len;
        curr[i]  = base[z] * (1 - y) + base[(z + 1) % base_len] * y;
    }
    return L;
}
#endif

#if defined(PEDALINATOR_FOUR_SIMPLE_NOTES_WITH_DMA) || defined(PEDALINATOR_DEFINITIVE_FOUR_NOTES_WITH_DMA)
#include "sample_22kHz_D2.h"
#include "sample_22kHz_D1.h"

uint32_t log_prevtime = 0;

/* // WORKING PYTHON ALGORITHM
 lp = LOOP_POINT_DATA[sample]["lp_start"]
ls = LOOP_POINT_DATA[sample]["lp_stop"]
(frames, nframes) = soundfile.read(sample + ".wav")
attacco_frames = frames[:lp]
corpo_frames = frames[lp:ls]
dsem = 4
ratio = 2**(dsem/12)
IL = len(corpo_frames)
L = math.trunc(len(corpo_frames) / ratio)
corpo_frames_2 = [0] * L
for i in range(0, L):
    x = (i * ratio)
    y = x - math.trunc(x)
    z = math.trunc(x) % IL
    corpo_frames_2[i] = corpo_frames[z] * \
        (1 - y) + corpo_frames[(z + 1) % IL] * y */

int attacco_pitch_shifting(int16_t *curr, size_t curr_len, int16_t *base, size_t base_len, int dsem) {
    if (dsem == 0) {
        return 0;
    }
    double ratio = pow(2.0, dsem / 12.0);
    size_t L     = base_len / ratio;
    for (size_t i = 0; i < L; i++) {
        double x = i * ratio;
        size_t y = (size_t)x;
        size_t z = (size_t)x % base_len;
        curr[i]  = base[z] * (1 - y) + base[(z + 1) % base_len] * y;
    }
    return 0;
}

// returns the new current note length
size_t corpo_pitch_shifting(int16_t *curr, size_t curr_max_len, int16_t *base, size_t base_len, int dsem) {
    if (dsem == 0) {
        return 0;
    }
    long double ratio = powl(2.0, dsem / 12.0);
    size_t L          = (size_t)((double)base_len / ratio);
    if (curr_max_len < L) {
        // TODO: better error handling
        Error_Handler();
    }
    // TODO: try different interpolation functions from the arm_math.h library
    // arm_linear_interp_instance_f32 config;
    // arm_linear_interp_f32(&config, 0.0f);
    for (size_t i = 0; i < L; i++) {
        double x = (double)i * ratio;
        double y = x - (size_t)x;
        size_t z = (size_t)x % base_len;
        curr[i]  = base[z] * (1 - y) + base[(z + 1) % base_len] * y;
    }
    // TODO: implement also time_stretching
    return L;
}

inline size_t min(size_t x, size_t y) {
    return (x > y) ? (y) : (x);
}

volatile bool dma_transfer_completed = false;
volatile bool dma_is_transmitting    = false;
void pedalinator_dma_completed_transfer_cb(DMA_HandleTypeDef *const hdma) {
    if (hdma == &handle_GPDMA1_Channel11) {
        // print("pedalinator_dma_completed_transfer_cb callback called...\r\n");
        // dma_transfer_completed = true;
        // dma_is_transmitting    = false;
    }
}

void pedalinator_dma_error_cb(DMA_HandleTypeDef *const hdma) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    Error_Handler();
}

void pedalinator_dma_abort_cb(DMA_HandleTypeDef *const hdma) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    Error_Handler();
}

void pedalinator_dma_suspend_cb(DMA_HandleTypeDef *const hdma) {
    print("pedalinator_dma_suspend_cb callback called...\r\n");
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai == &hsai_BlockA1) {
        print("HAL_SAI_TxCpltCallback callback called...\r\n");
        dma_transfer_completed = true;
        dma_is_transmitting    = false;
    }
}

volatile bool half_transfer = true;

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai){
    if (hsai == &hsai_BlockA1) {
        print("HAL_SAI_TxHalfCpltCallback callback called...\r\n");
        half_transfer = true;
    }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
    return;  // TODO: fix this
    if (hsai == &hsai_BlockA1) {
        uint32_t prevtime = HAL_GetTick();
        while (1) {
            uint32_t ctime = HAL_GetTick();
            if ((ctime - prevtime) > 100) {
                prevtime = HAL_GetTick();
                HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
                HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
            }
        }
    }
}

size_t compose_note(int nstate, int16_t *current_note, size_t current_note_max_len) {
    memcpy(current_note, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L);
    return SAMPLE_D2_22KHZ_CORPO_L;
}

#endif

#ifdef PEDALINATOR_COM_VIRTUAL_PORT_EXAMPLE
uint32_t prevtime = 0;

void cdc_task(void) {
    const char i_am_alive_msg[] = "I am alive\r\n";
    if (tud_cdc_write_available() && (HAL_GetTick() - prevtime) > 500) {
        prevtime = HAL_GetTick();
        HAL_UART_Transmit(&huart1, (uint8_t *)i_am_alive_msg, sizeof(i_am_alive_msg), 100);
        tud_cdc_write_str(i_am_alive_msg);
        tud_cdc_write_flush();
    }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the System Power */
    SystemPower_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_GPDMA1_Init();
    MX_ICACHE_Init();
    MX_USART1_UART_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_ADC1_Init();
    MX_SAI1_Init();
    MX_TIM1_Init();
    MX_DAC1_Init();
    /* USER CODE BEGIN 2 */

#ifdef TEST_TIMER
    const char message[] = "Son vivo\r\n";
#endif

#ifdef TEST_SWITCH
    const char message[] = "Son vivo\r\n";
    uint32_t ptime       = HAL_GetTick();
    uint32_t current_time;
    GPIO_PinState pstate = GPIO_PIN_RESET;
#endif

#ifdef TEST_INTERRUPTS
    uint32_t firstts = HAL_GetTick();
#endif

#ifdef PEDALINATOR_PITCH_MODULATION

    int16_t attacco[MAX_ATTACCO_L]               = {0};
    int16_t corpo[MAX_CORPO_L]                   = {0};
    int16_t decay[MAX_DECAY_L]                   = {0};
    int16_t current_note[MAX_CURRENT_NOTE_L]     = {0};
    int16_t current_note_old[MAX_CURRENT_NOTE_L] = {0};

    size_t attacco_len          = BASE_ATTACCO_L;
    size_t corpo_len            = BASE_CORPO_L;
    size_t decay_len            = BASE_DECAY_L;
    size_t current_note_len     = BASE_CURRENT_NOTE_L;
    size_t current_note_old_len = BASE_CURRENT_NOTE_L;

    size_t loop_point = SAMPLE_22KHZ5_LOOP_POINT;

    memcpy(current_note, sample_22kHz5, sizeof(int16_t) * SAMPLE_22KHZ5_SIZE);
    memcpy(current_note_old, sample_22kHz5, sizeof(int16_t) * SAMPLE_22KHZ5_SIZE);

    print(
        "attacco_len=%lu, corpo_len=%lu, decay_len=%lu, current_note_len=%lu, "
        "current_note_old_len=%lu\r\n",
        attacco_len,
        corpo_len,
        decay_len,
        current_note_len,
        current_note_old_len);

    elaborate_note_components(current_note, current_note_len, attacco, attacco_len, corpo, corpo_len, decay, decay_len);
    int sem_add = 1;

#endif

#ifdef PEDALINATOR_ALL_NOTES_STORED
    int cn_idx                   = 5;
    int16_t current_decay[30000] = {0};
#endif

#ifdef PEDALINATOR_ADVANCED_PITCH_MODULATION
    int dsem = 0;
    // int16_t c_attacco[];
#define C_CORPO_MAX_L 50000
    int16_t c_corpo[C_CORPO_MAX_L] = {0};
    memcpy(c_corpo, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L * sizeof(int16_t));
    size_t c_corpo_len = SAMPLE_D2_22KHZ_CORPO_L;
    // int16_t c_decay[];
#endif

#ifdef PEDALINATOR_FOUR_SIMPLE_NOTES_WITH_DMA
#define C_CORPO_MAX_L 50000
    int16_t c_corpo[C_CORPO_MAX_L] = {0};
    memset(c_corpo, 0, C_CORPO_MAX_L * sizeof(int16_t));
    size_t c_corpo_len = 0;

    int16_t c_adder[C_CORPO_MAX_L] = {0};
    memcpy(c_adder, sample_D1_22kHz_corpo, SAMPLE_D1_22KHZ_CORPO_L * sizeof(int16_t));
    // size_t c_adder_len = SAMPLE_D2_22KHZ_CORPO_L;
    int pstate = 0;

    HAL_StatusTypeDef result;

    result = HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel11, HAL_DMA_XFER_CPLT_CB_ID, pedalinator_dma_completed_transfer_cb);
    if (result != HAL_OK) {
        Error_Handler();
    }

    result = HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel11, HAL_DMA_XFER_ERROR_CB_ID, pedalinator_dma_error_cb);
    if (result != HAL_OK) {
        Error_Handler();
    }

    result = HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel11, HAL_DMA_XFER_ABORT_CB_ID, pedalinator_dma_abort_cb);
    if (result != HAL_OK) {
        Error_Handler();
    }

    result = HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel11, HAL_DMA_XFER_SUSPEND_CB_ID, pedalinator_dma_suspend_cb);
    if (result != HAL_OK) {
        Error_Handler();
    }
#endif

#ifdef PEDALINATOR_DEFINITIVE_FOUR_NOTES_WITH_DMA
    int16_t c_attacco[C_ATTACCO_MAX_L];
    memset(c_attacco, 0, C_ATTACCO_MAX_L * sizeof(int16_t));
    size_t c_attacco_len = 0;

    int16_t c_corpo[C_CORPO_MAX_L];
    memset(c_corpo, 0, C_CORPO_MAX_L * sizeof(int16_t));
    size_t c_corpo_len = 0;

    int16_t c_decay[C_DECAY_MAX_L];
    memset(c_decay, 0, C_DECAY_MAX_L * sizeof(int16_t));
    size_t c_decay_len = 0;

    int16_t current_note[CURRENT_NOTE_L];
    memset(current_note, 0, CURRENT_NOTE_L * sizeof(int16_t));
    size_t current_note_len = 0;

    int16_t buffer0_sai[CURRENT_NOTE_L];
    memset(buffer0_sai, 0, CURRENT_NOTE_L * sizeof(int16_t));
    size_t buffer0_sai_len = 0;

    int16_t buffer1_sai[CURRENT_NOTE_L];
    memset(buffer1_sai, 0, CURRENT_NOTE_L * sizeof(int16_t));
    size_t buffer1_sai_len = 0;

    buffer0_sai_len = SAMPLE_D2_22KHZ_CORPO_L;
    buffer1_sai_len = SAMPLE_D2_22KHZ_CORPO_L;
    memcpy(buffer0_sai, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L);
    memcpy(buffer1_sai, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L);

    int16_t *doublebuffer_sai[2]    = {buffer0_sai, buffer1_sai};
    size_t *doublebuffer_sai_len[2] = {&buffer0_sai_len, &buffer1_sai_len};

    bool ready_to_play_note = true;
    bool has_to_play_note   = false;
    bool active_buffer_sai  = 0;
    int pstate              = 0b1111;
    HAL_StatusTypeDef result;

    result = HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel11, HAL_DMA_XFER_CPLT_CB_ID, pedalinator_dma_completed_transfer_cb);
    if (result != HAL_OK) {
        Error_Handler();
    }

    result = HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel11, HAL_DMA_XFER_ERROR_CB_ID, pedalinator_dma_error_cb);
    if (result != HAL_OK) {
        Error_Handler();
    }

    result = HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel11, HAL_DMA_XFER_ABORT_CB_ID, pedalinator_dma_abort_cb);
    if (result != HAL_OK) {
        Error_Handler();
    }

    result = HAL_DMA_RegisterCallback(&handle_GPDMA1_Channel11, HAL_DMA_XFER_SUSPEND_CB_ID, pedalinator_dma_suspend_cb);
    if (result != HAL_OK) {
        Error_Handler();
    }
#endif

#ifdef PEDALINATOR_COM_VIRTUAL_PORT_EXAMPLE
    tusb_init();
    tud_init(BOARD_TUD_RHPORT);
#endif

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
#ifdef PEDALINATOR_COM_VIRTUAL_PORT_EXAMPLE
        tud_task();  // device task
        led_blinking_task();
        // cdc_task();

#endif

#ifdef PEDALINATOR_PITCH_MODULATION
        if (semitone_up) {
            // active_framebuffer = !active_framebuffer;
            // change_semitone(current_note[active_framebuffer], 1);
            semitone_up = false;
        }

        if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET) {
            HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t *)attacco, attacco_len, 3000);
            while (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET) {
                HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t *)corpo, corpo_len, 3000);
            }
            HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t *)decay, decay_len, 3000);

            int res = change_semitone(
                current_note,
                current_note_len,
                current_note_old,
                current_note_old_len,
                sem_add,
                attacco_len,
                corpo_len,
                decay_len,
                loop_point);
            ++sem_add;
            if (res < 0) {
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                Error_Handler();
            }
            res = elaborate_note_components(current_note, current_note_len, attacco, attacco_len, corpo, corpo_len, decay, decay_len);
            if (res < 0) {
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
                Error_Handler();
            }
            // memcpy(current_note_old, current_note, sizeof(int16_t) *
            // current_note_len);
            print(
                "attacco_len=%lu, corpo_len=%lu, decay_len=%lu, "
                "current_note_len=%lu, current_note_old_len=%lu\r\n",
                attacco_len,
                corpo_len,
                decay_len,
                current_note_len,
                current_note_old_len);
        }
#endif

#ifdef PEDALINATOR_ALL_NOTES_STORED
        if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET) {
            elaborate_decay(corpi[cn_idx], corpi_l[cn_idx], current_decay, corpi_l[cn_idx] / 2);

            HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t *)attacchi[cn_idx], attacchi_l[cn_idx], 3000);
            while (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET) {
                HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t *)corpi[cn_idx], corpi_l[cn_idx], 3000);
            }
            HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t *)current_decay, corpi_l[cn_idx] / 2, 3000);
            cn_idx++;
            cn_idx %= PEDALINATOR_N_NOTES;
        }
#endif

#ifdef PEDALINATOR_ADVANCED_PITCH_MODULATION
        if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET) {
            while (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET) {
                HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t *)c_corpo, c_corpo_len, 10000);
            }
            dsem--;
            c_corpo_len = corpo_pitch_shifting(c_corpo, C_CORPO_MAX_L, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, dsem);
            print("c_corpo_len = %lu\r\n", c_corpo_len);
        }
#endif

#ifdef PEDALINATOR_FOUR_SIMPLE_NOTES_WITH_DMA
        /*
    SOMETHING WORKS!!!

    Check all the 4 buttons
    if not changed since the last state: ok
    elaborate the note to play accordingly to the four buttons
    TODO: probably some sort of phase shifting will be necessary AND surely some
    sort of time stretching will be also needed. Now we will test cutting the
    minimum of the stuff

    The final version will make use of the DMA to offload the audio stuff and
    allow the audio to flow with no interruptions
    */
        GPIO_PinState n1 = HAL_GPIO_ReadPin(NOTA1_GPIO_Port, NOTA1_Pin);
        GPIO_PinState n2 = HAL_GPIO_ReadPin(NOTA2_GPIO_Port, NOTA2_Pin);
        GPIO_PinState n3 = HAL_GPIO_ReadPin(NOTA3_GPIO_Port, NOTA3_Pin);
        GPIO_PinState n4 = HAL_GPIO_ReadPin(NOTA4_GPIO_Port, NOTA4_Pin);
        int nstate       = n1 | n2 << 1 | n3 << 2 | n4 << 3;
        if (nstate != pstate) {
            pstate      = nstate;
            c_corpo_len = SIZE_MAX;
            memset(c_corpo, 0, C_CORPO_MAX_L * sizeof(int16_t));
            if (n1 == GPIO_PIN_RESET) {
                memcpy(c_corpo, sample_D1_22kHz_corpo, SAMPLE_D1_22KHZ_CORPO_L * sizeof(int16_t));
                c_corpo_len = SAMPLE_D1_22KHZ_CORPO_L;
            }
            if (n2 == GPIO_PIN_RESET) {
                size_t to_add = corpo_pitch_shifting(c_adder, C_CORPO_MAX_L, sample_D1_22kHz_corpo, SAMPLE_D1_22KHZ_CORPO_L, 2);
                c_corpo_len   = min(c_corpo_len, to_add);
                for (size_t i = 0; i < to_add && i < c_corpo_len; i++) {
                    c_corpo[i] += c_adder[i];
                }
            }
            if (n3 == GPIO_PIN_RESET) {
                size_t to_add = corpo_pitch_shifting(c_adder, C_CORPO_MAX_L, sample_D1_22kHz_corpo, SAMPLE_D1_22KHZ_CORPO_L, 4);
                c_corpo_len   = min(c_corpo_len, to_add);
                for (size_t i = 0; i < to_add && i < c_corpo_len; i++) {
                    c_corpo[i] += c_adder[i];
                }
            }
            if (n4 == GPIO_PIN_RESET) {
                size_t to_add = corpo_pitch_shifting(c_adder, C_CORPO_MAX_L, sample_D1_22kHz_corpo, SAMPLE_D1_22KHZ_CORPO_L, 5);
                c_corpo_len   = min(c_corpo_len, to_add);
                for (size_t i = 0; i < to_add && i < c_corpo_len; i++) {
                    c_corpo[i] += c_adder[i];
                }
            }
        }
        if (c_corpo_len != SIZE_MAX) {
            HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)c_corpo, (uint16_t)c_corpo_len);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            while (!dma_transfer_completed)
                ;
            dma_transfer_completed = false;
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            HAL_SAI_DMAStop(&hsai_BlockA1);
        }

#endif

#ifdef PEDALINATOR_DEFINITIVE_FOUR_NOTES_WITH_DMA
        GPIO_PinState n1 = HAL_GPIO_ReadPin(NOTA1_GPIO_Port, NOTA1_Pin);
        GPIO_PinState n2 = HAL_GPIO_ReadPin(NOTA2_GPIO_Port, NOTA2_Pin);
        GPIO_PinState n3 = HAL_GPIO_ReadPin(NOTA3_GPIO_Port, NOTA3_Pin);
        GPIO_PinState n4 = HAL_GPIO_ReadPin(NOTA4_GPIO_Port, NOTA4_Pin);
        int nstate       = n1 | (n2 << 1) | (n3 << 2) | (n4 << 3);
#define PINSTATE_TO_INT(state) (state == GPIO_PIN_SET ? 1 : 0)
        if (HAL_GetTick() - log_prevtime > 500) {
            print(
                "%d|%d|%d|%d; dma_transfer_completed=%d\r\n",
                PINSTATE_TO_INT(n1),
                PINSTATE_TO_INT(n2),
                PINSTATE_TO_INT(n3),
                PINSTATE_TO_INT(n4),
                (int)dma_transfer_completed);
            log_prevtime = HAL_GetTick();
        }

        __disable_irq();
        if (dma_transfer_completed || (!has_to_play_note && dma_is_transmitting)) {
            HAL_SAI_DMAStop(&hsai_BlockA1);
            dma_transfer_completed = false;
            dma_is_transmitting    = false;
            ready_to_play_note     = true;
            // ok to start another note
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        }
        // se qua scatta l'interrupt della callback siamo fregati
        if (ready_to_play_note && has_to_play_note) {
            ready_to_play_note  = false;
            dma_is_transmitting = true;
            HAL_SAI_Transmit_DMA(
                &hsai_BlockA1, (uint8_t *)doublebuffer_sai[active_buffer_sai], (uint16_t) * (doublebuffer_sai_len[active_buffer_sai]));
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        }
        __enable_irq();

        if (pstate == 0b1111 && nstate == 0b1111) {
            // np to np
            // ensure that no note is played
            has_to_play_note = false;
        }
        // else if (pstate == 0b1111 && nstate != 0b1111) { // Covered in the last case
        // }
        else if (pstate != 0b1111 && nstate == 0b1111) {
            // p to np
            // stop at the next iteration
            has_to_play_note = false;
        } else if (pstate == nstate) {
            // p same note
            // continue with the same buffer
            has_to_play_note = true;
        } else {
            // np to p
            // p different note
            // construct the note in the inactive buffer and then swap the buffer at the next iteration
            has_to_play_note                           = true;
            active_buffer_sai                          = !active_buffer_sai;
            *(doublebuffer_sai_len[active_buffer_sai]) = compose_note(nstate, doublebuffer_sai[active_buffer_sai], CURRENT_NOTE_L);
        }
        pstate = nstate;
#endif

#ifdef TEST_INTERRUPTS
        while (HAL_GetTick() - firstts < 3000)
            ;  // wait 5 seconds

        __disable_irq();
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
                          GPIO_PIN_SET);  // notify that interrupts are disabled
        HAL_UART_Transmit(&huart1, (uint8_t *)disable_it_msg, strlen(disable_it_msg), 100);

        GPIO_PinState button_pressed = GPIO_PIN_RESET;
        do {
            button_pressed = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
        } while (button_pressed == GPIO_PIN_RESET);  // wait for the button to be pressed

        button_pressed = GPIO_PIN_SET;
        do {
            button_pressed = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
        } while (button_pressed == GPIO_PIN_SET);  // wait for the button to be pressed

        button_pressed = GPIO_PIN_RESET;
        do {
            button_pressed = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
        } while (button_pressed == GPIO_PIN_RESET);  // wait for the button to be pressed

        button_pressed = GPIO_PIN_SET;
        do {
            button_pressed = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
        } while (button_pressed == GPIO_PIN_SET);  // wait for the button to be pressed

        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
                          GPIO_PIN_RESET);  // notify that interrupts are enabled again
        HAL_UART_Transmit(&huart1, (uint8_t *)enable_it_msg, strlen(enable_it_msg), 100);
        __enable_irq();
        while (1)
            ;
#endif

#ifdef TEST_SWITCH
        GPIO_PinState switch_state = HAL_GPIO_ReadPin(USER_SWITCH_GPIO_Port, USER_SWITCH_Pin);
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState            = RCC_LSI_ON;
    RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_0;
    RCC_OscInitStruct.LSIDiv              = RCC_LSI_DIV1;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLMBOOST       = RCC_PLLMBOOST_DIV4;
    RCC_OscInitStruct.PLL.PLLM            = 3;
    RCC_OscInitStruct.PLL.PLLN            = 10;
    RCC_OscInitStruct.PLL.PLLP            = 2;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 1;
    RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLLVCIRANGE_1;
    RCC_OscInitStruct.PLL.PLLFRACN        = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_PCLK3;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void) {
    HAL_PWREx_EnableVddIO2();

    /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
    HAL_PWREx_DisableUCPDDeadBattery();
    /* USER CODE BEGIN PWR */
    /* USER CODE END PWR */
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
