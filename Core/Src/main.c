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
#include "spi.h"
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
#include <inttypes.h>

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

#if defined(PEDALINATOR_ADVANCED_PITCH_MODULATION) || defined(PEDALINATOR_FOUR_SIMPLE_NOTES_WITH_DMA) || \
    defined(PEDALINATOR_DEFINITIVE_FOUR_NOTES_WITH_DMA)

#include "sample_22kHz_D2.h"

#if defined(PEDALINATOR_PHASE_VOCODER_ENABLED)


size_t phase_vocoder(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, int dsem) {
    
    // // Apply Hann window
    // hann_window(hann, FRAME_SIZE);
    // for (int i = 0; i < FRAME_SIZE; i++) {
    //     signal[i] *= hann[i];
    // }

    // // Perform FFT
    // fft(signal, FRAME_SIZE);    

    int16_t fft_buf[CURRENT_NOTE_L];

    const arm_cfft_instance_q15 cfft_instance = { 
        .fftLen = target_len,                   /**< length of the FFT. */
        .pTwiddle = NULL, // const q15_t *pTwiddle;             /**< points to the Twiddle factor table. */
        .pBitRevTable = NULL, // const uint16_t *pBitRevTable;      /**< points to the bit reversal table. */
        .bitRevLength = 0             /**< bit reversal table length. */
    };

    const arm_rfft_instance_q15 fft_instance = {
        .fftLenReal = target_len,              /**< length of the real FFT. */
        .ifftFlagR = 0,                        /**< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
        .bitReverseFlagR = 0,                  /**< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
        .twidCoefRModifier = 0,                /**< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
        .pTwiddleAReal = NULL, // const q15_t *pTwiddleAReal;   /**< points to the real twiddle factor table. */
        .pTwiddleBReal = NULL, // const q15_t *pTwiddleBReal;   /**< points to the imag twiddle factor table. */
        .pCfft = &cfft_instance, // const arm_cfft_instance_q15 *pCfft;    /**< points to the complex FFT instance. */
    };

    arm_rfft_q15(&fft_instance, base_note, fft_buf);

    // Time-stretch: Modify phase and overlap
    // [Details omitted for brevity—phase unwrapping, modification]

    const arm_cfft_instance_q15 inverse_cfft_instance = { 
        .fftLen = target_len,                   /**< length of the FFT. */
        .pTwiddle = NULL, // const q15_t *pTwiddle;             /**< points to the Twiddle factor table. */
        .pBitRevTable = NULL, // const uint16_t *pBitRevTable;      /**< points to the bit reversal table. */
        .bitRevLength = 0             /**< bit reversal table length. */
    };

    const arm_rfft_instance_q15 inverse_fft_instance = {
        .fftLenReal = target_len,              /**< length of the real FFT. */
        .ifftFlagR = 0,                        /**< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
        .bitReverseFlagR = 1,                  /**< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
        .twidCoefRModifier = 0,                /**< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
        .pTwiddleAReal = NULL, // const q15_t *pTwiddleAReal;   /**< points to the real twiddle factor table. */
        .pTwiddleBReal = NULL, // const q15_t *pTwiddleBReal;   /**< points to the imag twiddle factor table. */
        .pCfft = &cfft_instance, // const arm_cfft_instance_q15 *pCfft;    /**< points to the complex FFT instance. */
    };

    arm_rfft_q15(&inverse_fft_instance, fft_buf, target_note);

    // // Overlap and Add (OLA)
    // // [Details omitted for brevity]
    
}


size_t attacco_pitch_shifting(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, int dsem) {
    return phase_vocoder(target_note, target_len, sample_D2_22kHz_attacco, SAMPLE_D2_22KHZ_ATTACCO_L, dsem);
}

size_t corpo_pitch_shifting(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, int dsem) {
    return phase_vocoder(target_note, target_len, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, dsem);
}

size_t decay_pitch_shifting(int16_t *target_note, size_t target_len, int16_t *base_note, size_t base_note_len, int dsem) {
    // take base note and pitch shift
    size_t len = phase_vocoder(target_note, target_len, sample_D2_22kHz_attacco, SAMPLE_D2_22KHZ_ATTACCO_L, dsem);
    // then apply decay effect
    double TAU = (((double)target_len) / 15.0);
    for (size_t idx = 0; idx < target_len; idx++) {
        target_note[idx]  = target_note[idx] * exp(-(idx / TAU));
    }
    return target_len;
}

#else
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
        if (curr_max_len < base_len) {
            return 0;
        }
        memcpy(curr, base, base_len * sizeof(int16_t));
        return base_len;
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
    // arm_linear_interp_instance_f32 instance = {
    //     .nValues = L,
    //     .pYData = base, // technically double = float64
    //     .x1 = 1,
    //     .xSpacing = 1
    // };
    for (size_t i = 0; i < L; i++) {
        double x = (double)i * ratio;
        double y = x - (size_t)x;
        size_t z = (size_t)x % base_len;
        curr[i]  = base[z] * (1 - y) + base[(z + 1) % base_len] * y;
    }
    // TODO: implement also time_stretching
    return L;
}
#endif // PEDALINATOR_PHASE_VOCODER_ENABLED

#endif // PEDALINATOR_ADVANCED_PITCH_MODULATION or PEDALINATOR_FOUR_SIMPLE_NOTES_WITH_DMA or PEDALINATOR_DEFINITIVE_FOUR_NOTES_WITH_DMA

#if defined(PEDALINATOR_FOUR_SIMPLE_NOTES_WITH_DMA) || defined(PEDALINATOR_DEFINITIVE_FOUR_NOTES_WITH_DMA)

uint32_t log_prevtime = 0;

typedef enum {
    note_c,
    note_c_sharp,
    note_d,
    note_e_flat,
    note_e,
    note_f,
    note_f_sharp,
    note_g,
    note_g_sharp,
    note_a,
    note_b_flat,
    note_b,
    n_notes,
} note_names_t;

static const char note_names[n_notes][3] = {
    "C",
    "C#",
    "D",
    "Eb",
    "E",
    "F",
    "F#",
    "G",
    "G#",
    "A",
    "Bb",
    "B",
};

inline size_t min(size_t x, size_t y) {
    return (x > y) ? (y) : (x);
}

static volatile bool sai_transfer_completed      = false;
static volatile bool sai_is_transmitting         = false;
static volatile bool sai_half_transfer_completed = false;

void pedalinator_dma_completed_transfer_cb(DMA_HandleTypeDef *const hdma) {
    if (hdma == &handle_GPDMA1_Channel11) {
        /***
         * // NOT NEEDED
         * print("pedalinator_dma_completed_transfer_cb callback called...\r\n");
         * dma_transfer_completed = true;
         * dma_is_transmitting    = false; 
        */
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
        sai_transfer_completed = true;
        sai_is_transmitting    = false;
    }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai == &hsai_BlockA1) {
        print("HAL_SAI_TxHalfCpltCallback callback called...\r\n");
        sai_half_transfer_completed = true;
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

static inline float hanning_window(size_t i, size_t N) {
    return 0.5f * (1.0f - cosf(2.0f * PI * (float)i / (float)N));
}

/***
 * Applying a window crossfade to smoothen the sound wave
 * VERY POOR RESULTS!!!
 */
size_t apply_window_crossfading(int16_t *current_note, size_t current_note_length) {
    // crossfading algorithm
    // output[i] = w[i] × sample2[i] + (1−w[i]) × sample1[i]
    size_t crossfading_len = current_note_length / 48;
    for (size_t isample = 0; isample < crossfading_len; isample++) {
        float res = hanning_window(isample, crossfading_len) * (float)current_note[isample] +
                    (1.0f - hanning_window(isample, crossfading_len)) * (float)current_note[current_note_length - isample - 1];
        current_note[isample] = current_note[current_note_length - isample - 1] = (int16_t)res;
    }
    return current_note_length;
}

/***
 * The algorithm could have one flaw: it should be averaging the edges of the sample
 * VERY POOR RESULTS!!!
 */
size_t apply_linear_crossfading(int16_t *current_note, size_t current_note_length) {
    // output[i]=(1−Ni​)×sample1[i]+Ni​×sample2[i]
    size_t N = current_note_length / 24;
    for (size_t i = 0; i < N; i++) {
        float res       = (1 - (float)i / (float)N) * (float)current_note[i] + 1.0f / (float)N * (float)current_note[N - i - 1];
        current_note[i] = current_note[current_note_length - i - 1] = (uint16_t)res;
    }
    return current_note_length;
}

// size_t time_stretching(tmp_adder, tmp_adder_len, chosen_time_stretching_target) {
//     return 1;
// }

void smooth_loop(q15_t *buffer, int length, int len)
{
    int fade_len = len;
    for (int i = 0; i < fade_len; i++) {
        float fade_out = (float)(fade_len - i) / fade_len;
        float fade_in  = (float)i / fade_len;

        int end_idx = length - fade_len + i;
        float blended = buffer[end_idx] * fade_out + buffer[i] * fade_in;
        buffer[i] = (q15_t)blended;  // sostituisci l’inizio con il crossfade
    }
}

#define SMOOTH_LEN 128   // number of samples for crossfade (tweak as needed)

void smooth_loop2(q15_t* note, size_t len)
{
    if (len < 2 * SMOOTH_LEN)
        return; // buffer too short

    q15_t fade_in[SMOOTH_LEN];
    q15_t fade_out[SMOOTH_LEN];
    q15_t tmp[SMOOTH_LEN];

    // Generate linear ramp: fade_in = [0 .. 1], fade_out = [1 .. 0]
    for (int i = 0; i < SMOOTH_LEN; i++) {
        float32_t w = (float32_t)i / (float32_t)(SMOOTH_LEN - 1);
        fade_in[i]  = (q15_t)(w * 32767.0f);
        fade_out[i] = (q15_t)((1.0f - w) * 32767.0f);
    }

    // Multiply tails by fades
    // tmp = start * fade_out + end * fade_in
    for (int i = 0; i < SMOOTH_LEN; i++) {
        q15_t a = note[i];
        q15_t b = note[len - SMOOTH_LEN + i];
        q15_t fa = fade_out[i];
        q15_t fb = fade_in[i];

        // weighted sum using Q15 multiply-accumulate
        q31_t mix;
        mix  = (q31_t)a * fa; // Q15 * Q15 -> Q30
        mix += (q31_t)b * fb;
        mix >>= 15;           // back to Q15
        tmp[i] = (q15_t)__SSAT(mix, 16);
    }

    // Write smoothed transition into both ends
    memcpy(note, tmp, SMOOTH_LEN * sizeof(q15_t));
    memcpy(note + len - SMOOTH_LEN, tmp, SMOOTH_LEN * sizeof(q15_t));

    // Optional: low-pass smoothing of the boundary region
    // Apply a simple FIR with CMSIS-DSP
    static const q15_t lp_coeffs[5] = {
        6554, 13107, 19660, 13107, 6554 // ~Hamming low-pass kernel (sum = 0.999)
    };
    q15_t state[5 + SMOOTH_LEN - 1];
    arm_fir_instance_q15 S;
    arm_fir_init_q15(&S, 5, (q15_t *)lp_coeffs, state, SMOOTH_LEN);
    arm_fir_q15(&S, tmp, tmp, SMOOTH_LEN);
    memcpy(note, tmp, SMOOTH_LEN * sizeof(q15_t));
    memcpy(note + len - SMOOTH_LEN, tmp, SMOOTH_LEN * sizeof(q15_t));
}

#define SMOOTH3_LEN 128

void smooth3_loop(q15_t *note, size_t len) {
    if (len < 2 * SMOOTH3_LEN) return;

    for (int i = 0; i < SMOOTH3_LEN; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * 3.1415926f * i / (SMOOTH3_LEN - 1)));
        q15_t a = note[i];
        q15_t b = note[len - SMOOTH3_LEN + i];
        float sm = (float)a * (1.0f - w) + (float)b * w;
        note[i] = (q15_t)sm;
        note[len - SMOOTH3_LEN + i] = (q15_t)sm;
    }
}

/**
 * @brief Given the set of keys, it composes the waveform data to be played via I2S
 * 
 * @param nstate The bitset for the current set of keys pressed
 * @param pstate The bitset for the previous set of keys pressed
 * @param current_note Frame of the note
 * @param current_note_max_len Maximum length for the note frame
 * @return size_t 
 */
size_t compose_note(unsigned int nstate, unsigned int pstate, int16_t *current_note, size_t current_note_max_len) {
    uint32_t keep_notes = nstate & pstate;      // do the corpo
    uint32_t new_notes  = nstate ^ keep_notes;  // do the attacco
    uint32_t old_notes  = pstate ^ keep_notes;  // do the rilascio
    int16_t tmp_adder[CURRENT_NOTE_L];
    char display_notes_buf[BUFSIZ];
    size_t display_ptr = 0;

    memset(current_note, 0, CURRENT_NOTE_L * sizeof(current_note[0]));
    size_t n_notes = 0;
    size_t sofar = CURRENT_NOTE_L;

    for (size_t isem = 0; isem < 12; isem++) {
        if ((new_notes >> isem) & 1 || (keep_notes >> isem) & 1) {
            
#if 0
            #define N_MAGIC_COEFFS (9)
            // const static double magic_coeffs[N_MAGIC_COEFFS] = {1.0, 0.8, 0.6, 0.4, 0.3, 0.25, 0.2, 0.15, 0.1};
            const static q15_t magic_scaling[N_MAGIC_COEFFS] = {100, 80, 60, 40, 30, 25, 20, 15, 10};
            // const static q15_t magic_scaling[N_MAGIC_COEFFS] = {1, 2, 2, 3, 3, 4, 5, 6, 10};
#else
            #define N_MAGIC_COEFFS (6)
            // const static double magic_coeffs[N_MAGIC_COEFFS] = {1.0, 0.3, 0.1, 0.05, 0.02, 0.01};
            const static double magic_scaling[N_MAGIC_COEFFS] = {100, 30, 10, 5, 2, 1};
            // const static q15_t magic_scaling[N_MAGIC_COEFFS] = {1.0, 3, 10.0, 20.0, 50.0, 100.0};
#endif
            #define N_NOTE_FREQS (30)
            const static double note_frequencies[30] = {32.7032, 34.64783346745091, 36.70810085827109, 38.89087812335699, 41.20345007892202, 43.65353471889347, 46.249308972999806, 48.9994359965135, 51.913094082726424, 55.00000729465056, 58.270477918174294, 61.73542084498391, 65.4064, 69.29566693490182, 73.41620171654218, 77.78175624671398, 82.40690015784403, 87.30706943778694, 92.49861794599961, 97.998871993027, 103.82618816545286, 110.00001458930112, 116.54095583634859, 123.47084168996786, 130.8128, 138.59133386980363, 146.83240343308432, 155.56351249342796, 164.81380031568807, 174.61413887557384};
            
            const double f0 = note_frequencies[isem];
            q15_t sub_adder[CURRENT_NOTE_L];
            memset(sub_adder, 0, CURRENT_NOTE_L * sizeof(sub_adder[0]));
            memset(tmp_adder, 0, CURRENT_NOTE_L * sizeof(tmp_adder[0]));


            // il problema è la compatibilità tra q15_t e double... andrebbe scalato tutto e trattati come q15_t
            for (q15_t h = 0; h < N_MAGIC_COEFFS; h++) {
                double tmp = 2.0*3.1412*f0;
                for (q15_t iv = 0; iv < CURRENT_NOTE_L; iv++) {
                    sub_adder[iv] = arm_sin_q15(iv*(q15_t)(tmp*((double)h + 1.0)));
                }
                arm_scale_q15(sub_adder, magic_scaling[h], 4, sub_adder, CURRENT_NOTE_L);
                arm_add_q15(tmp_adder, sub_adder, tmp_adder, CURRENT_NOTE_L);
            }

            sofar = CURRENT_NOTE_L - CURRENT_NOTE_L % ((size_t) (22050.0 / f0));
            
            arm_add_q15(current_note, tmp_adder, current_note, CURRENT_NOTE_L);

            n_notes++;
            int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%s ", note_names[note_d + isem]);
            display_ptr += to_add;
        }
    }
    /***
     * Working on the smooth looping
     */
    // for (double i = 0.0; i < 100.0; i++) {
    //     size_t li = retval - i;
    //     size_t fi = i;
    //     q15_t le = current_note[li];
    //     q15_t fe = current_note[fi];
    //     current_note[li] = (q15_t) ((double) le * (i / 100.0) +  (double) fe * (100.0 - i / 100.0));
    //     current_note[fi] = (q15_t) ((double) fe * (i / 100.0) +  (double) le * (100.0 - i / 100.0));
    // }
    //smooth_loop2(current_note, retval);
    // int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%lu %lu %lu P %u N %u", keep_notes, new_notes, old_notes, pstate, nstate);
    // display_ptr += to_add;
    // smooth_loop(current_note, retval, 1000);
    // smooth3_loop(current_note, retval);

    // extern q15_t hamming_window[CURRENT_NOTE_L];
    // arm_scale_q15(current_note, 1, -8, current_note, CURRENT_NOTE_L);
    // arm_mult_q15(current_note, hamming_window, current_note, CURRENT_NOTE_L);
    // arm_scale_q15(current_note, 1, -8, current_note, CURRENT_NOTE_L);
    
    // ok this works but it fades too much to silence...
    // const size_t FOO = 2000;
    // for (int i = 0; i < FOO; i++) {
    //     current_note[CURRENT_NOTE_L - i] = current_note[CURRENT_NOTE_L - i] * ((double)i / (double) FOO);
    //     current_note[i] = current_note[i] * ((double)i / (double) FOO);
    // }

    lcd_1602a_write_text(display_notes_buf);
    print("\r\n");

    return sofar;
}

size_t compose_note2(unsigned int nstate, unsigned int pstate, int16_t *current_note, size_t current_note_max_len) {
    // placeholder

    // memcpy(current_note, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L * sizeof(uint16_t));
    // return SAMPLE_D2_22KHZ_CORPO_L;
    // return apply_window_crossfading(current_note, SAMPLE_D2_22KHZ_CORPO_L); // doesn't sound good...

    /***
     * pstate 0110111
     * nstate 1010011
     * 
     * keep_n 0010011
     * new_ns 1000100
     * old_ns 0100100
     */
    uint32_t keep_notes = nstate & pstate;      // do the corpo
    uint32_t new_notes  = nstate ^ keep_notes;  // do the attacco
    uint32_t old_notes  = pstate ^ keep_notes;  // do the rilascio
    int16_t tmp_adder[CURRENT_NOTE_L];
    int16_t tmp_time_stretcher[CURRENT_NOTE_L];
    size_t tmp_adder_len = 0;
    size_t tmp_time_stretcher_len = 0;
    char display_notes_buf[BUFSIZ];
    size_t display_ptr = 0;

    memset(current_note, 0, current_note_max_len);

    // if (HAL_GetTick() - log_prevtime > 500) {
    //     print(
    //         "%d|%d|%d|%d|%d; \r\n",
    //         (new_notes >> 0) & 1 || (keep_notes >> 0) & 1,
    //         (new_notes >> 1) & 1 || (keep_notes >> 1) & 1,
    //         (new_notes >> 2) & 1 || (keep_notes >> 2) & 1,
    //         (new_notes >> 3) & 1 || (keep_notes >> 3) & 1,
    //         (new_notes >> 4) & 1 || (keep_notes >> 4) & 1);
    //     log_prevtime = HAL_GetTick();
    // }

    size_t ctst = SAMPLE_D2_22KHZ_CORPO_L; // chosen_time_stretching_target
    size_t n_notes = 0;

#if defined(PEDALINATOR_PHASE_VOCODER_ENABLED)

    for (size_t isem = 0; isem < 12; isem++) {
        if ((new_notes >> isem) & 1) {
            attacco_pitch_shifting(tmp_adder, ctst, sample_D2_22kHz_attacco, SAMPLE_D2_22KHZ_ATTACCO_L, isem);
            // TODO: check ctst == return value of pitch shifting
            arm_add_q15(current_note, tmp_adder, current_note, ctst);
        } else if ((keep_notes >> isem) & 1) {
            corpo_pitch_shifting(tmp_adder, ctst, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, isem);
            // TODO: check ctst == return value of pitch shifting
            arm_add_q15(current_note, tmp_adder, current_note, ctst);
        } 
        /* 
            else if ((old_notes >> isem) & 1) {
                decay_pitch_shifting(tmp_adder, ctst, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, isem);
                // TODO: check ctst == return value of pitch shifting
                arm_add_q15(current_note, tmp_adder, current_note, ctst);
        } 
        */

        if ((new_notes >> isem) & 1 && (keep_notes >> isem) & 1) {
            int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%s ", note_names[note_d + isem]);
            display_ptr += to_add;
        }
    }

    lcd_1602a_write_text(display_notes_buf);

    if (n_notes > 1)
        arm_scale_q15(current_note, 0xFFFF / n_notes, 0, current_note, current_note_max_len);
        
    return ctst;

#else


    for (size_t isem = 0; isem < 12; isem++) {
        
        /* 
            As it should be...
            if ((nstate >> isem) & 1) {
                int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%s ", note_names[note_d + isem]);
                display_ptr += to_add;
            }

            if ((new_notes >> isem) & 1) {
                tmp_adder_len        = corpo_pitch_shifting(tmp_adder, CURRENT_NOTE_L, sample_D2_22kHz_attacco, SAMPLE_D2_22KHZ_ATTACCO_L, isem);
                current_note_max_len = min(current_note_max_len, tmp_adder_len);
                arm_add_q15(current_note, tmp_adder, current_note, current_note_max_len);
                n_notes++;
            } else if ((keep_notes >> isem) & 1) {
                tmp_adder_len         = corpo_pitch_shifting(tmp_adder, CURRENT_NOTE_L, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, isem);
                current_note_max_len = min(current_note_max_len, tmp_adder_len);
                arm_add_q15(current_note, tmp_adder, current_note, current_note_max_len);
                n_notes++;
            } 
        */

        if (((new_notes >> isem) & 1) || ((keep_notes >> isem) & 1)) {
            tmp_adder_len         = corpo_pitch_shifting(tmp_adder, CURRENT_NOTE_L, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, isem);
            current_note_max_len = min(current_note_max_len, tmp_adder_len);
            arm_add_q15(current_note, tmp_adder, current_note, current_note_max_len);
            n_notes++;
            int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%s ", note_names[note_d + isem]);
            display_ptr += to_add;
        } 
    }

    // int to_add = snprintf(display_notes_buf + display_ptr, BUFSIZ - display_ptr, "%lu %lu %lu P %u N %u", keep_notes, new_notes, old_notes, pstate, nstate);
    // display_ptr += to_add;
    lcd_1602a_write_text(display_notes_buf);

    print("\r\n");

    // consider whether to include it
    // if (n_notes > 1)
    // arm_scale_q15(current_note, 8, 0, current_note, current_note_max_len);

    return current_note_max_len;
#endif
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
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */

/* 
    #define TN 12
    q15_t a[TN] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    // q15_t c[TN] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    q15_t c[TN] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    char bstr[BUFSIZ]; size_t to_add = 0; size_t ptr = 0;
    while(1) {
        arm_add_q15(a, c, c, TN);
        for (size_t i = 0; i < TN; i++) {
            to_add = snprintf(bstr + ptr, BUFSIZ - ptr - 1, "%" PRIu16 ", ", c[i]);
            ptr += to_add;
        }
        print("%s\r\n", bstr);
        memset(bstr, 0, ptr + 1);
        ptr = 0;
        HAL_Delay(500);
    }
 */

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

#if defined(LCD_1602A_ENABLED)
    lcd_1602a_init();
    // lcd_1602a_test();
    lcd_1602a_write_text("pedalinator v1");
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

    int16_t buffer1_sai[CURRENT_NOTE_L];
    memset(buffer1_sai, 0, CURRENT_NOTE_L * sizeof(int16_t));

    // memcpy(buffer0_sai, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L);
    // memcpy(buffer1_sai, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L);
    // memcpy(buffer0_sai + sizeof(uint8_t) * SAMPLE_D2_22KHZ_CORPO_L, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L);
    // memcpy(buffer1_sai + sizeof(uint8_t) * SAMPLE_D2_22KHZ_CORPO_L, sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L);

    // int16_t *doublebuffer_sai[2]    = {buffer0_sai, buffer1_sai};
    // size_t doublebuffer_sai_len[2] =  {SAMPLE_D2_22KHZ_CORPO_L, SAMPLE_D2_22KHZ_CORPO_L};

    int16_t *doublebuffer_sai[2]   = {buffer0_sai, buffer1_sai};
    size_t doublebuffer_sai_len[2] = {0, 0};

#define N_PEDALINATOR_NOTES (5) // MAX 31 NOTES
    static GPIO_TypeDef *button_notes_ports[N_PEDALINATOR_NOTES] = {
        NOTA5_GPIO_Port,
        NOTA1_GPIO_Port,
        NOTA2_GPIO_Port,
        NOTA3_GPIO_Port,
        NOTA4_GPIO_Port,
    };

    const static uint16_t button_notes_pins[N_PEDALINATOR_NOTES] = {
        NOTA5_Pin,
        NOTA1_Pin,
        NOTA2_Pin,
        NOTA3_Pin,
        NOTA4_Pin,
    };

    volatile bool ready_to_play_note   = true;
    volatile bool has_to_play_note     = false;
    volatile bool has_to_change_note   = false;
    uint8_t active_buffer_sai = 0;
    // bool first_half = true;
    // uint8_t inactive_buffer_sai  = 1;
    uint32_t pstate          = (1 << N_PEDALINATOR_NOTES) - 1;
    // uint32_t buffer_notes[2] = {0};
    HAL_StatusTypeDef result = HAL_OK;

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
            // memcpy(current_note_old, current_note, sizeof(int16_t) * current_note_len);
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

#warning TODO Software debounce
        // GPIO_PinState n1 = !HAL_GPIO_ReadPin(NOTA1_GPIO_Port, NOTA1_Pin);
        // GPIO_PinState n2 = !HAL_GPIO_ReadPin(NOTA2_GPIO_Port, NOTA2_Pin);
        // GPIO_PinState n3 = !HAL_GPIO_ReadPin(NOTA3_GPIO_Port, NOTA3_Pin);
        // GPIO_PinState n4 = !HAL_GPIO_ReadPin(NOTA4_GPIO_Port, NOTA4_Pin);
        // GPIO_PinState n5 = !HAL_GPIO_ReadPin(NOTA5_GPIO_Port, NOTA5_Pin);
        uint32_t nstate = 0;
        for (size_t inote = 0; inote < N_PEDALINATOR_NOTES; inote++) {
            GPIO_PinState state = !HAL_GPIO_ReadPin(button_notes_ports[inote], button_notes_pins[inote]);
            nstate |= (state << inote);
        }

        // uint32_t nstate  = n1 | (n2 << 1) | (n3 << 2) | (n4 << 3) | (n5 << 4);

#define PINSTATE_TO_INT(state) (state == GPIO_PIN_SET ? 1 : 0)
#if defined(PEDALINATOR_DEBUG_ENABLED)
        if (HAL_GetTick() - log_prevtime > 500) {
            print(
                "%d|%d|%d|%d|%d; sai_half_transfer_completed=%d\r\n",
                (nstate >> 0) & 1,
                (nstate >> 1) & 1,
                (nstate >> 2) & 1,
                (nstate >> 3) & 1,
                (nstate >> 4) & 1,
                (int)sai_half_transfer_completed);
            log_prevtime = HAL_GetTick();
        }
#endif

        /* 
            WARNING: beware of the SAI interrupts, maybe disabling IRQ is not needed
            __disable_irq();
            if (sai_transfer_completed) {
                // HAL_SAI_DMAStop(&hsai_BlockA1); // maybe not needed
                sai_transfer_completed = false;
                // sai_is_transmitting    = false; // not needed
                ready_to_play_note     = true;
                // ok to start another note
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            }
            if (sai_half_transfer_completed && sai_is_transmitting) {
            // if (sai_transfer_completed || (!has_to_play_note && sai_is_transmitting)) {

                HAL_SAI_DMAStop(&hsai_BlockA1);
                sai_half_transfer_completed = false;
                ready_to_play_note     = true;
                // ok to start another note
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
            }
            if (ready_to_play_note && has_to_play_note) {
                ready_to_play_note  = false;
                sai_is_transmitting = true;
                active_buffer_sai = !active_buffer_sai;
                HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)doublebuffer_sai[active_buffer_sai], (uint16_t) * (doublebuffer_sai_len[active_buffer_sai]));
                HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            }

            sai_half_transfer_completed
            sai_transfer_completed
            sai_is_transmitting
            ready_to_play_note
            has_to_play_note
        */

        // __disable_irq();

        if (!has_to_play_note && sai_is_transmitting) {
            // stop the sound
            HAL_SAI_DMAStop(&hsai_BlockA1);
            sai_half_transfer_completed = false;
            sai_transfer_completed      = false;
            sai_is_transmitting         = false;

            ready_to_play_note = true;
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        } else if (has_to_play_note && sai_transfer_completed) {
            // HAL_SAI_DMAStop(&hsai_BlockA1);
            /* 
                bool update_not_active_buffer = false;
                // different note
                if (buffer_notes[active_buffer_sai] != pstate) {
                    update_not_active_buffer = true;
                    active_buffer_sai = !active_buffer_sai;
                    buffer_notes[!active_buffer_sai] = buffer_notes[active_buffer_sai] = pstate;
                } 
            */

            sai_transfer_completed = false;
            sai_is_transmitting    = true;
            HAL_SAI_Transmit_DMA(
                &hsai_BlockA1, ((uint8_t *)doublebuffer_sai[active_buffer_sai]), (uint16_t)doublebuffer_sai_len[active_buffer_sai]);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

            /* 
                if (update_not_active_buffer) {
                    memcpy(doublebuffer_sai[!active_buffer_sai], doublebuffer_sai[active_buffer_sai], doublebuffer_sai_len[active_buffer_sai] * sizeof (int16_t));
                    update_not_active_buffer = false;
                } 
            */
        } else if (has_to_play_note && ready_to_play_note) {
            ready_to_play_note  = false;
            sai_is_transmitting = true;

            HAL_SAI_Transmit_DMA(
                &hsai_BlockA1, ((uint8_t *)doublebuffer_sai[active_buffer_sai]), (uint16_t)doublebuffer_sai_len[active_buffer_sai]);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        } else if (has_to_change_note) {
            has_to_change_note = false;
            ready_to_play_note = false;
            sai_is_transmitting = true;

            HAL_SAI_DMAStop(&hsai_BlockA1);
            HAL_SAI_Transmit_DMA(
                &hsai_BlockA1, ((uint8_t *)doublebuffer_sai[active_buffer_sai]), (uint16_t)doublebuffer_sai_len[active_buffer_sai]);
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        }

        // __enable_irq();

#define ALL_KEYS_RELEASED (0)
        if (pstate == ALL_KEYS_RELEASED && nstate == ALL_KEYS_RELEASED) {
            // np to np
            // ensure that no note is played
            lcd_1602a_clear_screen();
            has_to_play_note = false;
        }
        // else if (pstate == 0b1111 && nstate != 0b1111) { // Covered in the last case
        // }
        else if (pstate != ALL_KEYS_RELEASED && nstate == ALL_KEYS_RELEASED) {
            // p to np
            // stop at the next iteration
            has_to_play_note = false;
        } else if (pstate == nstate) {
            // p same note
            // continue with the same buffer
            has_to_play_note = true;
            // memcpy(doublebuffer_sai[!active_buffer_sai], doublebuffer_sai[active_buffer_sai], doublebuffer_sai_len[active_buffer_sai] * sizeof(int16_t));
        } else {
            // TODO: avoid interrupting the wave, either wait for the DMA to finish transmitting or start the note with an offset (very cool ngl)
            #warning avoid interrupting the wave 
            // np to p
            // p different note
            // construct the note in the inactive buffer and then swap the buffer at the next iteration
            // has_to_play_note                         = true;
            has_to_change_note = true;
            doublebuffer_sai_len[!active_buffer_sai] = compose_note2(nstate, pstate, doublebuffer_sai[!active_buffer_sai], CURRENT_NOTE_L);
            active_buffer_sai = !active_buffer_sai;
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
