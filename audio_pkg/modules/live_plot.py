#!/usr/bin/env python2.7

import pyaudio
import struct
import numpy as np
import matplotlib.pyplot as plt
from scipy.fftpack import fft
import time
from Tkinter import TclError

# Constants
CHUNK = 1024 * 8  # samples per frame
FORMAT = pyaudio.paInt16  # audio format (bytes per sample)
CHANNELS = 1  # single channel for microphone
RATE = 192000  # samples per second

# Create figure for plotting
fig, (ax1, ax2) = plt.subplots(2, figsize=(15, 7))

# PyAudio class instance
p = pyaudio.PyAudio()

# Stream object to get data from microphone
stream = p.open(
    format=FORMAT,
    channels=CHANNELS,
    rate=RATE,
    input=True,
    output=True,
    frames_per_buffer=CHUNK,
)

# Variables for plotting
x = np.arange(0, 2 * CHUNK, 2)  # samples (waveform)
xf = np.linspace(0, RATE, CHUNK)  # frequencies (spectrum)

# Create a line object with random data
line, = ax1.plot(x, np.random.rand(CHUNK), '-', lw=2)

# Create semilogx line for spectrum
line_fft, = ax2.semilogx(xf, np.random.rand(CHUNK), '-', lw=2)

# Format waveform axes
ax1.set_title("AUDIO WAVEFORM")
ax1.set_xlabel("Samples")
ax1.set_ylabel("Volume")
ax1.set_ylim(0, 255)
ax1.set_xlim(0, 2 * CHUNK)
plt.setp(ax1, xticks=[0, CHUNK, 2 * CHUNK], yticks=[0, 128, 255])

# Format spectrum axes
ax2.set_title("FREQUENCY SPECTRUM")
ax2.set_xlim(20, RATE / 2)

print("Stream started")

# For measuring frame rate
frame_count = 0
start_time = time.time()

while True:
    try:
        # Binary data
        data = stream.read(CHUNK)

        # Convert data to integers, make np array, then offset it by 128
        data_int = struct.unpack(str(2 * CHUNK) + 'B', data)
        data_np = np.array(data_int, dtype='b')[::2] + 128

        # Update waveform line
        line.set_ydata(data_np)

        # Compute FFT and update spectrum line
        yf = fft(data_np - 128)  # FFT input needs to be zero-centered
        line_fft.set_ydata(np.abs(yf[0:CHUNK]) / (128 * CHUNK))

        # Update figure canvas
        fig.canvas.draw()
        fig.canvas.flush_events()
        frame_count += 1

    except TclError:
        # Calculate average frame rate
        frame_rate = frame_count / (time.time() - start_time)
        print("Stream stopped")
        print(f"Average frame rate = {frame_rate:.0f} FPS")
        break
