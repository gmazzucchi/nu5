// TODO: It would be cool to use Miniaudio instead of writing to a file
// and then call to the python script to generate the .wav file

#include <stdio.h>
#include "arm_math.h"
#include "suoninator_config.h"
#include "additive_synth_gen.h"

#define N_HW_KEYS (64)
#define N_HW_PEDAL_KEYS (64)

int main(void) {
    init_orchestra();
    orchestra_set_note(0, 28);
    orchestra_set_note(0, 40);
    orchestra_set_note(0, 44);
    orchestra_set_note(0, 47);

#define MAX_CURRENT_NOTE (65536)
    q15_t current_note[MAX_CURRENT_NOTE];
    const size_t current_note_len = MAX_CURRENT_NOTE;
    size_t effective_len = additive_synth_compose_note(current_note, current_note_len);
    FILE* outfile = fopen("note.tmp", "w+");
#define N_REPS 15
    for (size_t irep = 0; irep < N_REPS; irep++) {
        for (size_t i = 0; i < effective_len; i++) {
            fprintf(outfile, "%d\r\n", (int)current_note[i]);
        }
    }

    orchestra_unset_note(0, 28);
    orchestra_unset_note(0, 40);
    orchestra_unset_note(0, 44);
    orchestra_unset_note(0, 47);
    orchestra_set_note(0, 28);
    orchestra_set_note(0, 40);
    orchestra_set_note(0, 45);
    orchestra_set_note(0, 49);

    effective_len = additive_synth_compose_note(current_note, current_note_len);
    for (size_t irep = 0; irep < N_REPS; irep++) {
        for (size_t i = 0; i < effective_len; i++) {
            fprintf(outfile, "%d\r\n", (int)current_note[i]);
        }
    }

    orchestra_unset_note(0, 28);
    orchestra_unset_note(0, 40);
    orchestra_unset_note(0, 45);
    orchestra_unset_note(0, 49);
    orchestra_set_note(0, 28);
    orchestra_set_note(0, 40);
    orchestra_set_note(0, 44);
    orchestra_set_note(0, 47);

    effective_len = additive_synth_compose_note(current_note, current_note_len);
    for (size_t irep = 0; irep < N_REPS; irep++) {
        for (size_t i = 0; i < effective_len; i++) {
            fprintf(outfile, "%d\r\n", (int)current_note[i]);
        }
    }
    fflush(outfile);
    fclose(outfile);
    return 0;
}
