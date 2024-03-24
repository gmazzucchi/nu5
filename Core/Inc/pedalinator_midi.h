#ifndef PEDALINATOR_MIDI_H
#define PEDALINATOR_MIDI_H

#include <stdbool.h>
#include <stdint.h>

void led_blinking_task(void);
void midi_task(void);
void board_led_write(bool led_state);
uint32_t board_millis(void);

#endif // PEDALINATOR_MIDI_H

