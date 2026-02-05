#ifndef ORGAN_PRESETS_H
#define ORGAN_PRESETS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// similar to a double-reed instrument
extern const double ORGAN_PRESET_1[];
extern const size_t ORGAN_PRESET_1_SIZE;
// a dolce principal register
extern const double ORGAN_PRESET_2[];
extern const size_t ORGAN_PRESET_2_SIZE;
// soft register
// amps2 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1]
// mixture - principal
extern const double ORGAN_PRESET_3[];
extern const size_t ORGAN_PRESET_3_SIZE;

#endif // ORGAN_PRESETS_H

