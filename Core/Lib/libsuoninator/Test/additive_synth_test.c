#include <stdio.h>
#include "arm_math.h"
#include "wavegen.h"
#include "ped_config.h"

int main(void)
{
    // q15_t x = 8192;  // pi/2
    // q15_t y = arm_sin_q15(x);
    // printf("sin(pi/2) Q15 = %d\n", y);
    // printf("sin(pi/2) float = %.6f\n", y / 32768.0f);

    bool pstate_keys[N_HW_KEYS] = {0};
    bool nstate_keys[N_HW_KEYS] = {0};
    bool pstate_pedals[N_HW_PEDAL_KEYS] = {0};
    bool nstate_pedals[N_HW_PEDAL_KEYS] = {0};
    nstate_keys[28] = 1;
    nstate_keys[35] = 1;
    nstate_keys[40] = 1;
    nstate_keys[44] = 1;
    nstate_keys[47] = 1;
    nstate_keys[52] = 1;
#define MAX_CURRENT_NOTE (65536)
    q15_t current_note[MAX_CURRENT_NOTE];
    const size_t current_note_len = MAX_CURRENT_NOTE;
    size_t effective_len = compose_note(pstate_keys, nstate_keys, pstate_pedals, nstate_pedals, current_note, current_note_len);
    FILE* outfile = fopen("note.tmp", "w+");
#define N_REPS 15
    for (size_t irep = 0; irep < N_REPS; irep++) {
        for (size_t i = 0; i < effective_len; i++) {
            fprintf(outfile, "%d\r\n", (int)current_note[i]);
        }
    }    
    fflush(outfile);
    fclose(outfile);
    return 0;
}
