#ifndef USB_AUDIO_TEST_H
#define USB_AUDIO_TEST_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"

#ifndef AUDIO_SAMPLE_RATE
#define AUDIO_SAMPLE_RATE 48000
#endif

extern uint32_t sampFreq;
extern uint8_t clkValid;
// extern audio_control_range_4_n_t(1) sampleFreqRng;

#endif // USB_AUDIO_TEST_H
