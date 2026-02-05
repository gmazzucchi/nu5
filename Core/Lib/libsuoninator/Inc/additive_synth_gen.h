#ifndef WAVEGEN_H
#define WAVEGEN_H

#include "arm_math.h"

#include "suoninator_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
    bool was_initialized;
    uint8_t past_played_notes_all_instruments[SUONINATOR_N_INSTRUMENTS][(SUONINATOR_N_KEYS / 8) + 1];
    uint8_t new_played_notes_all_instruments[SUONINATOR_N_INSTRUMENTS][(SUONINATOR_N_KEYS / 8) + 1];
    const double* additive_synths_coeffs[SUONINATOR_N_INSTRUMENTS];
    size_t n_coeffs_per_instrument[SUONINATOR_N_INSTRUMENTS];
} orchestra_t;

void init_orchestra(void);
int orchestra_set_note(size_t instrument_idx, size_t note_idx);
int orchestra_unset_note(size_t instrument_idx, size_t note_idx);
size_t additive_synth_compose_note(int16_t* current_note, size_t max_current_note_len);

#endif // WAVEGEN_H
