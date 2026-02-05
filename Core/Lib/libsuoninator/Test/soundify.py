import wave
import struct

def write_wav_int16(filename, samples, fs):
    with wave.open(filename, 'w') as wav_file:
        n_channels = 1          # mono
        sampwidth = 2           # bytes (int16)
        framerate = fs
        n_frames = len(samples)
        comptype = "NONE"
        compname = "not compressed"

        wav_file.setparams((n_channels, sampwidth, framerate, n_frames, comptype, compname))

        # Pack samples as little-endian int16
        frames = struct.pack('<' + 'h' * len(samples), *samples)
        wav_file.writeframes(frames)


fs = 22000
infile = open('note.tmp', 'r')
samples = [int(i) for i in infile.readlines()]
infile.close()

write_wav_int16("output.wav", samples, fs)
