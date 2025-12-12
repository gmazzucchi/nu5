import numpy as np
from scipy.io.wavfile import write

# Parameters
samplerate = 44100
duration = 1
fftlen = int(4096)
fftlen = int(samplerate * duration)

# Additive parameters (harmonics)
f0 = 100
amps = [100, 30, 50, 40, 50, 10, 15, 20, 10, 10, 20]   # amplitudes
phases = np.zeros(len(amps))    # or choose random phases

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
signal = np.tile(signal, [400])

# Store WAV
write("ifft_additive.wav", samplerate, (signal * 32767).astype(np.int16))
print("saved ifft_additive.wav")
