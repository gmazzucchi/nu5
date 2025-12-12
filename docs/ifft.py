import numpy as np
from scipy.io.wavfile import write
from pydub import AudioSegment

samplerate = 44100          # CD quality
duration = 2.0              # seconds
f0 = 200                    # fundamental frequency (Hz)

additive_synthesis_coeffs = [100, 30, 10, 5, 2, 1]

t = np.linspace(0, duration, int(samplerate * duration), endpoint=False)
signal = np.zeros_like(t, dtype=float)

for h, amp in enumerate(additive_synthesis_coeffs, start=1):
    signal += amp * np.sin(2 * np.pi * h * f0 * t)
signal /= np.max(np.abs(signal))

wav_data = (signal * 32767).astype(np.int16)
write("additive.wav", samplerate, wav_data)

sound = AudioSegment(
    wav_data.tobytes(),
    frame_rate=samplerate,
    sample_width=2,
    channels=1
)
sound.export("additive.mp3", format="mp3")

print("Saved additive.wav and additive.mp3")
