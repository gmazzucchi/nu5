#include "organ_presets.h"

// similar to a double-reed instrument
const double ORGAN_PRESET_1[] = {0.1, 0.03, 0.05, 0.04, 0.05, 0.01, 0.015, 0.02, 0.02, 0.01, 0.02};
const size_t ORGAN_PRESET_1_SIZE = sizeof(ORGAN_PRESET_1) / sizeof(ORGAN_PRESET_1[0]);
// a dolce principal register
const double ORGAN_PRESET_2[] = {0.2, 0.1, 0, 0, 0.05, 0.1, 0.03, 0.02, 0.01};
const size_t ORGAN_PRESET_2_SIZE = sizeof(ORGAN_PRESET_2) / sizeof(ORGAN_PRESET_2[0]);
// soft register
// amps2 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1]
// mixture - principal
const double ORGAN_PRESET_3[] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
const size_t ORGAN_PRESET_3_SIZE = sizeof(ORGAN_PRESET_3) / sizeof(ORGAN_PRESET_3[0]);

