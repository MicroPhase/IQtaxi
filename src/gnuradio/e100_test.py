#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: v3.8.5.0-6-g57bd109d

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio.filter import firdes
import sip
from gnuradio import analog
from gnuradio import gr
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import iqtaxi
from gnuradio.qtgui import Range, RangeWidget

from gnuradio import qtgui

class e100_test(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Not titled yet")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "e100_test")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.tx_att = tx_att = 10
        self.samp_rate = samp_rate = 15.36e6
        self.rx_gain = rx_gain = 0
        self.lo_freq = lo_freq = 1000e6
        self.bb_freq = bb_freq = 100e3

        ##################################################
        # Blocks
        ##################################################
        self._tx_att_range = Range(0, 40, 1, 10, 200)
        self._tx_att_win = RangeWidget(self._tx_att_range, self.set_tx_att, 'tx_att', "counter_slider", float)
        self.top_layout.addWidget(self._tx_att_win)
        self._rx_gain_range = Range(0, 40, 1, 0, 200)
        self._rx_gain_win = RangeWidget(self._rx_gain_range, self.set_rx_gain, 'rx_gain', "counter_slider", float)
        self.top_layout.addWidget(self._rx_gain_win)
        self._lo_freq_range = Range(70e6, 6000e6, 1e6, 1000e6, 200)
        self._lo_freq_win = RangeWidget(self._lo_freq_range, self.set_lo_freq, 'lo_freq', "counter_slider", float)
        self.top_layout.addWidget(self._lo_freq_win)
        self._bb_freq_range = Range(1e3, 1000e3, 1e3, 100e3, 200)
        self._bb_freq_win = RangeWidget(self._bb_freq_range, self.set_bb_freq, 'bb_freq', "counter_slider", float)
        self.top_layout.addWidget(self._bb_freq_win)
        self.qtgui_sink_x_0 = qtgui.sink_c(
            1024, #fftsize
            firdes.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "", #name
            True, #plotfreq
            True, #plotwaterfall
            True, #plottime
            True #plotconst
        )
        self.qtgui_sink_x_0.set_update_time(1.0/10)
        self._qtgui_sink_x_0_win = sip.wrapinstance(self.qtgui_sink_x_0.pyqwidget(), Qt.QWidget)

        self.qtgui_sink_x_0.enable_rf_freq(False)

        self.top_layout.addWidget(self._qtgui_sink_x_0_win)
        self.iqtaxi_tx_sink_0 = iqtaxi.tx_sink("E100", "192.168.1.10", samp_rate, lo_freq, tx_att, 4096, False, 0)
        self.iqtaxi_rx_source_0 = iqtaxi.rx_source("E100", "192.168.1.10", samp_rate, lo_freq, rx_gain, 4096)
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, bb_freq, 1, 0, 0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.iqtaxi_tx_sink_0, 0))
        self.connect((self.iqtaxi_rx_source_0, 0), (self.qtgui_sink_x_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "e100_test")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_tx_att(self):
        return self.tx_att

    def set_tx_att(self, tx_att):
        self.tx_att = tx_att
        self.iqtaxi_tx_sink_0.set_attenuation(self.tx_att)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)
        self.iqtaxi_rx_source_0.set_sample_rate(self.samp_rate)
        self.iqtaxi_tx_sink_0.set_sample_rate(self.samp_rate)
        self.qtgui_sink_x_0.set_frequency_range(0, self.samp_rate)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.iqtaxi_rx_source_0.set_gain(self.rx_gain)

    def get_lo_freq(self):
        return self.lo_freq

    def set_lo_freq(self, lo_freq):
        self.lo_freq = lo_freq
        self.iqtaxi_rx_source_0.set_center_freq(self.lo_freq)
        self.iqtaxi_tx_sink_0.set_center_freq(self.lo_freq)

    def get_bb_freq(self):
        return self.bb_freq

    def set_bb_freq(self, bb_freq):
        self.bb_freq = bb_freq
        self.analog_sig_source_x_0.set_frequency(self.bb_freq)





def main(top_block_cls=e100_test, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    def quitting():
        tb.stop()
        tb.wait()

    qapp.aboutToQuit.connect(quitting)
    qapp.exec_()

if __name__ == '__main__':
    main()
