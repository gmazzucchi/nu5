from playsound import playsound
import numpy as np
from scipy.io.wavfile import write

# Parameters
samplerate = 44100
duration = 0.2
n_tiles = 5
fftlen = int(4096)
fftlen = int(samplerate * duration)
OUTPUT_FILEPATH = 'ifft_additive.wav'

def generate_sound(f0, amps, phases):
    # Create empty complex FFT spectrum
    spec = np.zeros(fftlen, dtype=np.complex128)

    # Fill FFT bins
    for h, amp in enumerate(amps, start=1):
        freq = h * f0                                  # harmonic frequency
        bin_index = int(freq * fftlen / samplerate)    # map Hz -> FFT bin

        A = amp / 2.0                                  # split energy into ±freq
        phase = phases[h-1]

        # complex coefficient for positive frequency
        spec[bin_index] = A * np.exp(1j * phase)
        # enforce Hermitian symmetry for real output
        spec[-bin_index] = A * np.exp(-1j * phase)

    # Inverse FFT → time signal
    signal = np.fft.ifft(spec).real

    # Normalize
    signal /= np.max(np.abs(signal))
    signal = np.tile(signal, [n_tiles])
    return signal


def gen_chords(f0, amps):
    phases = np.zeros(len(amps))

    sbase = generate_sound(f0 / 2, amps, phases)
    salto = generate_sound(f0 * 2, amps, phases)
    s0 = generate_sound(f0, amps, phases)
    f1 = f0 * 2**(4/12)
    s1 = generate_sound(f1, amps, phases)
    f2 = f0 * 2**(7/12)
    s2 = generate_sound(f2, amps, phases)
    tmp1 = sbase + salto + s0 + s1 + s2
    tmp1 /= np.max(np.abs(tmp1))

    s0 = generate_sound(f0, amps, phases)
    f1 = f0 * 2**(5/12)
    s1 = generate_sound(f1, amps, phases)
    f2 = f0 * 2**(9/12)
    s2 = generate_sound(f2, amps, phases)
    tmp2 = sbase + salto + s0 + s1 + s2
    tmp2 /= np.max(np.abs(tmp2))

    signal = np.concatenate((tmp1, tmp2, tmp1, tmp1))
    return signal


if __name__ == '__main__':
    f0 = 221

    # lontanamente simile a una doppia ancia
    amps1 = [0.1, 0.03, 0.05, 0.04, 0.05, 0.01, 0.015, 0.02, 0.02, 0.01, 0.02]

    # un principale dolce    
    amps2 = [0.2, 0.1, 0, 0, 0.05, 0.1, 0.03, 0.02, 0.01]

    # registro dolce
    # amps2 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.1]

    # un misto tra un principale e un ripieno
    amps3 = [0.5] * 20

    signal1 = gen_chords(f0, amps1)
    signal2 = gen_chords(f0, amps2)
    signal3 = gen_chords(f0, amps3)
    signal = signal1 + signal2 + signal3
    signal /= np.max(np.abs(signal))

    write(OUTPUT_FILEPATH, samplerate, (signal * 32767).astype(np.int16))

    print(f"Saved file {OUTPUT_FILEPATH}")
    playsound(OUTPUT_FILEPATH)
    