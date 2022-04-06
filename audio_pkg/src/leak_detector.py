#!/usr/bin/env python2.7
#!/usr/bin env bash
import pyaudio
import struct
import math
import time
import sys
from scipy.fftpack import fft
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.signal import butter, lfilter


#  Audio params
INITIAL_TAP_THRESHOLD = 0.01
FORMAT = pyaudio.paInt16
SHORT_NORMALIZE = 1.0 / 32768.0
CHANNELS = 1
RATE = 192000
INPUT_BLOCK_TIME = 0.05  # CHUNK 9600 samples
INPUT_FRAMES_PER_BLOCK = int(RATE * INPUT_BLOCK_TIME)


# if we get this many noisy blocks in a row, increase the threshold
OVERSENSITIVE = 15.0 / INPUT_BLOCK_TIME
# if we get this many quiet blocks in a row, decrease the threshold
UNDERSENSITIVE = 120.0 / INPUT_BLOCK_TIME
# if the noise was longer than this many blocks, it's not a 'tap'
MAX_TAP_BLOCKS = 0.15 / INPUT_BLOCK_TIME

MIN_FRQ = 35000  # Hz
MAX_FRQ = 45000  # Hz


# Filters
def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype="band")
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y


def get_rms(block):
    count = len(block) / 2
    FORMAT = "%dh" % (count)
    shorts = struct.unpack(FORMAT, block)

    # iterate over the block.
    sum_squares = 0.0
    shorts_normalized = [sample * SHORT_NORMALIZE for sample in shorts]

    # Apply high pass filter
    shorts_normalized = butter_bandpass_filter(
        shorts_normalized, MIN_FRQ, MAX_FRQ, RATE, order=5
    )

    sum_squares = [n * n for n in shorts_normalized]

    return math.sqrt(sum_squares / count)


class LeakDetector(object):
    def __init__(self):
        self.pa = pyaudio.PyAudio()
        self.stream = self.open_mic_stream()
        self.tap_threshold = INITIAL_TAP_THRESHOLD
        self.noisycount = MAX_TAP_BLOCKS + 1
        self.quietcount = 0
        self.errorcount = 0

        self.amp = 0
        self.freq = 0

        self.LEAK_THRESHOLD = 50
        self.count_ultrasound_frames = 0
        self.leak_detection_timeout = 0.1  # 100 ms

        # frequency part
        self.MIN_INDEX = 1750
        self.MAX_INDEX = 4800
        self.HZ_PER_SAMPLE = 20
        self.is_leak_detected = False
        self.NOISE_RMS_ALL_FREQS = 0.0005  # for all freqs 35kHz to 45KHz
        self.NOISE_RMS_40KHZ = 0.1

        self.MIN_FREQ = 35000  # Hz
        self.MAX_FREQ = 45000  # Hz

        self.global_time = time.time()

        self.leak_status = "unknown"
        self.is_leak_detected = False

    def stop(self):
        self.stream.close()

    def find_input_device(self):
        device_index = None
        for i in range(self.pa.get_device_count()):
            devinfo = self.pa.get_device_info_by_index(i)
            print("Device %d: %s" % (i, devinfo["name"]))

            for keyword in ["mic", "input"]:
                if keyword in devinfo["name"].lower():
                    print("Found an input: device %d - %s" % (i, devinfo["name"]))
                    device_index = i

        if device_index is None:
            print("No preffered input found; using default input device.")

        return device_index

    def open_mic_stream(self):
        device_index = self.find_input_device()

        self.stream = self.pa.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=INPUT_FRAMES_PER_BLOCK,
        )
        return self.stream

    def leakDetected(self):
        curr_time = time.time()

        elapsed = curr_time - self.global_time

        if elapsed < self.leak_detection_timeout:
            print("Found leak!!")
            self.global_time = time.time()
            self.leak_status = "Leak detected!"
            self.is_leak_detected = True

        else:
            print("Probably not a leak")
            self.leak_status = "unknown leak status"
            self.is_leak_detected = False
            self.global_time = time.time()
        self.global_time = time.time()

    

    def listen(self):
        try:
            block = self.stream.read(
                INPUT_FRAMES_PER_BLOCK, exception_on_overflow=False
            )  # Raw data bytes

        except IOError as e:
            self.errorcount += 1
            print("Error recording")
            self.noisycount = 1
            return

        amplitude = get_rms(block)
        self.amp = amplitude

        # print("Amplitude is: ",amplitude,"Threshold is: ",self.tap_threshold)
        if amplitude > self.tap_threshold:
            # noisy block
            self.quietcount = 0
            self.noisycount += 1

            if self.noisycount > OVERSENSITIVE:
                self.tap_threshold *= 1.1

            else:

                if 1 <= self.noisycount <= MAX_TAP_BLOCKS:
                    self.leakDetected()

                self.noisycount = 0
                self.quietcount += 1
                if self.quietcount > UNDERSENSITIVE:
                    self.tap_threshold *= 0.9

    def get_db(self):

        return db = 20 * math.log10(self.amp / self.NOISE_RMS_ALL_FREQS)
        

