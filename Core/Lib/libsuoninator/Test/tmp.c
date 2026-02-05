#include <stdio.h>
#include <stdlib.h>

// similar to a double-reed instrument
const static double ORGAN_PRESET_1[] = {0.1, 0.03, 0.05, 0.04, 0.05, 0.01, 0.015, 0.02, 0.02, 0.01, 0.02};
const static size_t ORGAN_PRESET_1_SIZE = sizeof(ORGAN_PRESET_1) / sizeof(ORGAN_PRESET_1[0]);
// a dolce principal register
const static double ORGAN_PRESET_2[] = {0.2, 0.1, 0, 0, 0.05, 0.1, 0.03, 0.02, 0.01};
const static size_t ORGAN_PRESET_2_SIZE = sizeof(ORGAN_PRESET_2) / sizeof(ORGAN_PRESET_2[0]);
// soft register
// amps2 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1]
// mixture - principal
const static double ORGAN_PRESET_3[] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
const static size_t ORGAN_PRESET_3_SIZE = sizeof(ORGAN_PRESET_3) / sizeof(ORGAN_PRESET_3[0]);


int main(int argc, char const *argv[])
{
    for (size_t i = 0; i < ORGAN_PRESET_3_SIZE; i++)
    {
        double a = i < ORGAN_PRESET_1_SIZE ? ORGAN_PRESET_1[i] : 0;
        double b = i < ORGAN_PRESET_2_SIZE ? ORGAN_PRESET_2[i] : 0;
        double c = i < ORGAN_PRESET_3_SIZE ? ORGAN_PRESET_3[i] : 0;
        printf("%f, ", a + b + c);
    }
    printf("\r\n");
    return 0;
}
