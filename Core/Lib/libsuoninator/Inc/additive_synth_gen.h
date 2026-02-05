#ifndef WAVEGEN_H
#define WAVEGEN_H

#include "arm_math.h"

#include "suoninator_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
    size_t n_instruments;
    uint8_t** past_played_notes;
    uint8_t** new_played_notes;
    size_t* n_keys_per_instrument;
    const double** additive_synths_coeffs;
    size_t* n_coeffs_per_instrument;
} orchestra_t;

size_t additive_synth_compose_note(int16_t* current_note, size_t max_current_note_len, const orchestra_t* orchestra);

#endif // WAVEGEN_H
