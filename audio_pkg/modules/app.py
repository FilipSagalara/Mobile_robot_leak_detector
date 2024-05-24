#!/usr/bin/env python2.7

import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import struct
import pyaudio
from scipy.fftpack import fft
import sys

# Importing the custom audio interface and processing modules
from audio_interface import AudioInterface
from audio_processing import AudioProcessing


class AudioStream(object):
    def __init__(self):
        self.mic = AudioInterface()
        self.apu = AudioProcessing()

        # PyQtGraph configuration
        pg.setConfigOptions(antialias=True)
        self.traces = {}
        self.app = QtGui.QApplication(sys.argv)
        self.win = pg.GraphicsWindow(title="Spectrum Analyzer")
        self.win.setWindowTitle("Spectrum Analyzer")
        self.win.setGeometry(5, 115, 1910, 1070)

        self.waveform = self._create_plot(
            title="WAVEFORM",
            row=1,
            col=1,
            x_labels=[(0, "0"), (4096, "4096"), (8192, "8192")],
            y_labels=[(0, "0"), (32768 / 2, str(int(32768 / 2))), (32768, "32768")]
        )

        self.spectrum = self._create_plot(
            title="SPECTRUM",
            row=2,
            col=1,
            x_labels=[
                (np.log10(10), "10"),
                (np.log10(100), "100"),
                (np.log10(1000), "1000"),
                (np.log10(22050), "22050")
            ],
            log_mode_x=True,
            y_range=(-4, 2),
            x_range=(np.log10(20), np.log10(22050))
        )

        # PyAudio configuration
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 192000
        self.CHUNK = 1024 * 8

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            output=True,
            frames_per_buffer=self.CHUNK,
        )

        # Waveform and spectrum x points
        self.x = np.arange(0, 2 * self.CHUNK, 2)
        self.f = np.linspace(0, self.RATE / 2, self.CHUNK / 2)

    def _create_plot(self, title, row, col, x_labels, y_labels=None, log_mode_x=False, y_range=None, x_range=None):
        """Helper function to create a plot with specific configurations."""
        x_axis = pg.AxisItem(orientation="bottom")
        x_axis.setTicks([x_labels])

        plot_args = {"bottom": x_axis}
        if y_labels:
            y_axis = pg.AxisItem(orientation="left")
            y_axis.setTicks([y_labels])
            plot_args["left"] = y_axis

        plot = self.win.addPlot(title=title, row=row, col=col, axisItems=plot_args)
        if log_mode_x:
            plot.setLogMode(x=True, y=True)
        if y_range:
            plot.setYRange(*y_range, padding=0)
        if x_range:
            plot.setXRange(*x_range, padding=0.005)

        return plot

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
            QtGui.QApplication.instance().exec_()

    def set_plotdata(self, name, data_x, data_y):
        if name in self.traces:
            self.traces[name].setData(data_x, data_y)
        else:
            pen_color = "c" if name == "waveform" else "m"
            self.traces[name] = self.waveform.plot(pen=pen_color, width=3) if name == "waveform" else self.spectrum.plot(pen=pen_color, width=3)

            if name == "waveform":
                self.waveform.setYRange(0, 32768, padding=0)
                self.waveform.setXRange(0, 2 * self.CHUNK, padding=0.005)
            if name == "spectrum":
                self.spectrum.setLogMode(x=True, y=True)
                self.spectrum.setYRange(-4, 2, padding=0)
                self.spectrum.setXRange(np.log10(20), np.log10(self.RATE / 2), padding=0.005)

    def update_from_mic(self):
        wf_data = self.mic.get_single_chunk()
        self.set_plotdata(name="waveform", data_x=self.x, data_y=wf_data)

        sp_data = fft(np.array(wf_data, dtype="int16"))
        sp_data = np.abs(sp_data[:self.CHUNK // 2]) * 2 / (128 * self.CHUNK)
        self.set_plotdata(name="spectrum", data_x=self.f, data_y=sp_data)

    def animation(self):
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update_from_mic)
        timer.start(50)
        self.start()


if __name__ == "__main__":
    audio_app = AudioStream()
    audio_app.animation()
