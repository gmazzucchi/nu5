#include "additive_synth_gen.h"
#include "organ_presets.h"

#include <stdio.h>

#define CHECK_INITIALIZATION() if(!orchestra.was_initialized) return 0;
    
static orchestra_t orchestra = {
    .was_initialized = false
};

void init_orchestra(void) {
    for (size_t i = 0; i < SUONINATOR_N_INSTRUMENTS; i++) {
        orchestra.additive_synths_coeffs[0] = (double*) ORGAN_PRESET_3;
        orchestra.n_coeffs_per_instrument[0] = ORGAN_PRESET_3_SIZE;
    }    
    orchestra.was_initialized = true;
}

int orchestra_set_preset(size_t instrument_idx, const double* synth_coeffs, size_t n_coeffs) {
    CHECK_INITIALIZATION();
    if (instrument_idx >= SUONINATOR_N_INSTRUMENTS) return 0;
    // orchestra.n_coeffs_per_instrument[]
    return 1;
}
int orchestra_set_note(size_t instrument_idx, size_t note_idx) {
    CHECK_INITIALIZATION();
    if (instrument_idx >= SUONINATOR_N_INSTRUMENTS || note_idx >= SUONINATOR_N_KEYS) return 0;
    size_t idx = note_idx / 8;
    size_t offset = note_idx % 8;
    orchestra.new_played_notes_all_instruments[instrument_idx][idx] |= (1<<offset);
    return 1;
}
int orchestra_unset_note(size_t instrument_idx, size_t note_idx) {
    CHECK_INITIALIZATION();
    if (instrument_idx >= SUONINATOR_N_INSTRUMENTS || note_idx >= SUONINATOR_N_KEYS) return 0;
    size_t idx = note_idx / 8;
    size_t offset = note_idx % 8;
    orchestra.new_played_notes_all_instruments[instrument_idx][idx] &= 0xFF ^ (1<<offset);
    return 1;
}

const static double FREQUENCY_TABLE[100] = {
    16.35, 17.32222, 18.35225, 19.44354, 20.59971, 21.82463, 23.12239, 24.49732, 25.95401, 27.49731, 29.13239, 30.86469, 32.7, 34.64444, 36.70451, 38.88707, 41.19942, 43.64926, 46.24478, 48.99464, 51.90801, 54.99463, 58.26478, 61.72938, 65.4, 69.28889, 73.40902, 77.77415, 82.39884, 87.29853, 92.48957, 97.98928, 103.81603, 109.98925, 116.52955, 123.45876, 130.8, 138.57777, 146.81804, 155.54829, 164.79767, 174.59705, 184.97913, 195.97857, 207.63206, 219.9785, 233.0591, 246.91752, 261.6, 277.15555, 293.63607, 311.09658, 329.59535, 349.19411, 369.95827, 391.95713, 415.26412, 439.957, 466.11821, 493.83504, 523.2, 554.31109, 587.27214, 622.19316, 659.19069, 698.38821, 739.91654, 783.91426, 830.52823, 879.91401, 932.23642, 987.67008, 1046.4, 1108.62218, 1174.54429, 1244.38633, 1318.38139, 1396.77642, 1479.83307, 1567.82853, 1661.05646, 1759.82802, 1864.47284, 1975.34016, 2092.8, 2217.24436, 2349.08857, 2488.77265, 2636.76277, 2793.55285, 2959.66614, 3135.65705, 3322.11292, 3519.65604, 3728.94567, 3950.68032, 4185.6, 4434.48873, 4698.17715, 4977.5453
};

arm_status compute_ifft(q15_t *current_note, const size_t note_fft_len) {
    arm_cfft_instance_q15 fft_instance;
    arm_status status = arm_cfft_init_q15(&fft_instance, note_fft_len);
    
    if (status != ARM_MATH_SUCCESS) return status;
    arm_cfft_q15(&fft_instance, current_note, 1, 1);
    /***
     * It feels so wrong to skip normalization after ifft computation...
     */
    // arm_scale_q15(current_note, 1, log2(note_fft_len), current_note, 2*note_fft_len);
    for (size_t isample = 0; isample < note_fft_len; isample++) {
        current_note[isample] = current_note[2*isample];
    }    
    return ARM_MATH_SUCCESS;
}

void add_frequency_components_for_a_note(q15_t* spec, const size_t note_fft_len, const double* organ_preset, 
                                         const size_t organ_preset_size, const size_t kidx) {
    double total_gain = 0;
    for (size_t h = 0; h < organ_preset_size; h++) {
        total_gain += organ_preset[h];
    }
    for (size_t h = 0; h < organ_preset_size; h++) {
        double f0 = FREQUENCY_TABLE[kidx] * (h + 1);
        const size_t N = note_fft_len;
        size_t bin = (size_t) round(f0*h*N/AUDIO_FREQUENCY_HZ);
        if (bin >= N/2) break;
        /***
         * I feel removing scaling to total_gain is wrong, 
         * because sum(organ_preset) > 1 and should be normalized in theory
         */
        // q15_t A = (q15_t) (organ_preset[h] * (32768.0) / total_gain);
        q15_t A = (q15_t) (organ_preset[h] * (32768.0));

        spec[2*bin + 0] += A;     // Real
        spec[2*bin + 1] += 0;     // Imaginary
        spec[2*(N-bin) + 0] += A; // Symmetric real
        spec[2*(N-bin) + 1] += 0; // Symmetric imaginary
    }
}
/**
 * TODO: to fix, this is broken
size_t sample_sinusoid(bool *nstate_keys, bool *nstate_pedals, q15_t* current_note, size_t current_note_len) {
    float f0 = 221;
    float fs = AUDIO_FREQUENCY_HZ;
    float period = fs / f0;
    for (size_t is = 0; is < current_note_len; is++) {
        // TODO: this is the wrong part surely
        q15_t x = (q15_t)((is / (2.0f * PI)) * 32768.0f);   
        current_note[is] = arm_sin_q15(x);
    }
    return current_note_len;
}
*/

size_t additive_synth_compose_note(int16_t* current_note, size_t max_current_note_len) {
    CHECK_INITIALIZATION();
#if 0
/**
 * To test the sound player mechanism, first compose a sample sinusoid in function of the key states...
 */
    return sample_sinusoid(nstate_keys, nstate_pedals, current_note, current_note_len);
#endif
    const size_t note_fft_len = 4096U;
    
    if (max_current_note_len < note_fft_len * 2) {
        return 0;
    }
    memset(current_note, 0, note_fft_len * 2 * sizeof(current_note[0]));

    for (size_t iinstr = 0; iinstr < SUONINATOR_N_INSTRUMENTS; iinstr++) {
        for (size_t inote = 0; inote < ((SUONINATOR_N_KEYS/8)+1); inote++) {
            for (size_t ibit = 0; ibit < 8; ibit++) {
                if (orchestra.new_played_notes_all_instruments[iinstr][inote] & (1 << ibit)) {
                    printf("Adding note %d at bit %d\r\n", (int)inote, (int)ibit);
                    add_frequency_components_for_a_note(current_note, note_fft_len, orchestra.additive_synths_coeffs[iinstr], 
                                                        orchestra.n_coeffs_per_instrument[iinstr], inote * 8 + ibit);
                }
            }
        }
    }    
    compute_ifft(current_note, note_fft_len);
    
    q15_t max_value = 0;
    uint32_t max_idx;
    arm_max_q15(current_note, note_fft_len, &max_value, &max_idx);

#if 0
    /***
     * A futura memoria, 
     * arm_scale_q15 deve moltiplicare due q15_t, che sono in pratica due int16_t.
     * Per fare ciò li mette temporaneamente in un q31_t, li moltiplica insieme e poi tiene 
     * i 15 bit piu significativi.
     * Se moltiplico per 5 e basta, sicuro mi servono anche i bit non significativi,
     * altrimenti il risultato viene troncato. Per fare ciò devo shiftare a sinistra
     * di 15 dopo la moltiplicazione e allora diventano bit significativi e il risultato
     * viene preservato.
     * Bisogna ragionare con gli int16_t e non con il float scalato da -1 a 1.
     * Lascio qua il codice di debug come reperto.
     */
#include <stdio.h>
    printf("The max value was %d at index %d\r\n", (int)max_value, (int)max_idx);
    q15_t scale_factor = (1 << 15) / max_value;
    arm_scale_q15(current_note, 1 << 15, 5, current_note, current_note_len);
    for (size_t i = 0; i < note_fft_len; i++) {
        current_note[i] = current_note[i] << 5;
    }
    arm_shift_q15(current_note, 1, current_note, note_fft_len);
    arm_scale_q15(current_note, n_scaling, 15, current_note, note_fft_len);
    arm_max_q15(current_note, note_fft_len, &max_value, &max_idx);
    printf("The max value is now %d at index %d\r\n", (int)max_value, (int)max_idx);
#endif

#define MAX_IDEAL_VALUE_A_RECIA (10000U)
    if (max_value != 0) {
        int8_t n_scaling = MAX_IDEAL_VALUE_A_RECIA > max_value 
                            ? (MAX_IDEAL_VALUE_A_RECIA / max_value) 
                            : (0 - (max_value / MAX_IDEAL_VALUE_A_RECIA));
        arm_scale_q15(current_note, n_scaling, 15, current_note, note_fft_len);
    }
    return note_fft_len;
}

