#!/usr/bin/env python3
import sys
import time
import struct
import binascii
from collections import deque

import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# ────────────────────────────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 460800
TIMEOUT     = 0.1   # seconds

SOF         = 0xAA
PT_ENCODER  = 0x01
PT_COMBINED = 0x03
# ────────────────────────────────────────────────────────────────────────────────

def read_exact(ser, n: int):
    """Read exactly n bytes or return None on timeout."""
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


class SpeedPlotter(QtWidgets.QWidget):
    def __init__(self,
                 port=SERIAL_PORT,
                 baud=BAUD_RATE,
                 timeout=TIMEOUT,
                 bufsize=500,
                 read_hz=200,
                 plot_hz=30):
        super().__init__()
        self.setWindowTitle("Real-time Speed Plot")

        # Serial port
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        time.sleep(0.1)
        self.start_time = time.time()

        # Buffers for speed
        self.speed_t = deque(maxlen=bufsize)
        self.speed_d = deque(maxlen=bufsize)

        # UI: single speed plot
        layout = QtWidgets.QVBoxLayout(self)
        self.plot = pg.PlotWidget(title="Speed (°/s)")
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel('left', 'Speed', units='°/s')
        self.plot.setLabel('bottom', 'Time', units='s')
        self.curve = self.plot.plot(pen='c')
        layout.addWidget(self.plot)

        # Optimize drawing
        self.curve.setDownsampling(auto=True, method='mean')
        self.curve.setClipToView(True)

        # Timers: read at read_hz, redraw at plot_hz
        self.read_timer = QtCore.QTimer(self)
        self.read_timer.timeout.connect(self.read_serial)
        self.read_timer.start(int(1000 / read_hz))

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.redraw)
        self.plot_timer.start(int(1000 / plot_hz))

    def read_serial(self):
        """Read as many full packets as available and stash speed into buffers."""
        now = time.time() - self.start_time

        while True:
            b = self.ser.read(1)
            if not b:
                break
            if b[0] != SOF:
                continue

            hdr = read_exact(self.ser, 2)
            if not hdr:
                break
            pkt_type, length = struct.unpack('<BB', hdr)

            payload = read_exact(self.ser, length)
            crc_bytes = read_exact(self.ser, 2)
            if payload is None or crc_bytes is None:
                break

            # we ignore CRC check here for speed

            if pkt_type == PT_COMBINED and length == 16:
                # Combined: [angle_h, avg_speed_h, yaw_h, pitch_h]
                _, avg_h, _, _ = struct.unpack('<iiii', payload)
                speed = avg_h / 100.0
            elif pkt_type == PT_ENCODER and length == 8:
                # Encoder: [angle_h, speed_h]
                _, speed_h = struct.unpack('<ii', payload)
                speed = speed_h / 100.0
            else:
                # skip any other packet
                continue

            # append to buffers
            self.speed_t.append(now)
            self.speed_d.append(speed)

    def redraw(self):
        """Repaint the speed curve at a human-viewable rate."""
        if self.speed_t:
            self.curve.setData(self.speed_t, self.speed_d)
            now = time.time() - self.start_time
            # keep last 1 second visible
            self.plot.setXRange(now - 1.0, now)

    def closeEvent(self, ev):
        self.ser.close()
        ev.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = SpeedPlotter()
    w.resize(600, 400)
    w.show()
    sys.exit(app.exec_())
