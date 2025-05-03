#!/usr/bin/env python3
import sys
import time
import re
from collections import deque

import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# ────────────────────────────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 460800
TIMEOUT     = 0.1   # seconds

# ASCII format:
#   printf("[Encoder] angle = %.2f°, speed = %.2f°/s\n", angleDeg, speedDeg);
ENC_PATTERN = re.compile(
    r"\[Encoder\]\s+angle\s*=\s*([-0-9.]+)°,\s*speed\s*=\s*([-0-9.]+)°/s"
)

def compute(f_velocity, f_angle):
    """ASCII command to set speed & angle on the device."""
    return "#{0}:{1:.2f}:{2:.2f};;\r\n".format(13, f_velocity, f_angle)
# ────────────────────────────────────────────────────────────────────────────────

class SerialPlotter(QtWidgets.QWidget):
    def __init__(self,
                 port=SERIAL_PORT,
                 baud=BAUD_RATE,
                 timeout=TIMEOUT,
                 bufsize=500,
                 poll_hz=30,
                 cmd_hz=5,
                 initial_speed=0.0,
                 initial_angle=0.0):
        super().__init__()
        self.setWindowTitle(f"Real-time Encoder Plot @ {baud} baud ({poll_hz} Hz data, {cmd_hz} Hz cmd)")

        # Track last angle so we can include it with speed commands
        self.last_angle = initial_angle
        # Desired speed to send repeatedly
        self.desired_speed = initial_speed

        # Open serial port
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        time.sleep(0.1)
        self.start_time = time.time()

        # Send initial speed/angle command
        init_msg = compute(self.desired_speed, self.last_angle)
        print("Sending initial compute():", init_msg.strip())
        self.ser.write(init_msg.encode())

        # Data buffers
        self.angle_t = deque(maxlen=bufsize)
        self.angle_d = deque(maxlen=bufsize)
        self.speed_t = deque(maxlen=bufsize)
        self.speed_d = deque(maxlen=bufsize)

        # ─ UI ─────────────────────────────────────────────────────────────────────
        layout = QtWidgets.QVBoxLayout(self)

        # — Control panel —
        ctrl = QtWidgets.QHBoxLayout()
        layout.addLayout(ctrl)
        ctrl.addWidget(QtWidgets.QLabel("Set speed (°/s):"))
        self.speed_box = QtWidgets.QDoubleSpinBox()
        self.speed_box.setRange(-360.0, 360.0)
        self.speed_box.setSingleStep(1.0)
        self.speed_box.setValue(self.desired_speed)
        ctrl.addWidget(self.speed_box)
        self.speed_box.valueChanged.connect(self.on_speed_change)

        # Show the latest reported speed from the device
        self.lbl_speed = QtWidgets.QLabel("Latest device speed: — °/s")
        layout.addWidget(self.lbl_speed)

        # — Plotting area —
        pw = pg.GraphicsLayoutWidget()
        layout.addWidget(pw)

        # Angle plot
        self.p1 = pw.addPlot(row=0, col=0, title="Angle (°)")
        self.p1.showGrid(x=True, y=True)
        self.p1.setYRange(0, 360, padding=0)

        # Speed plot (shares X axis)
        self.p2 = pw.addPlot(row=1, col=0, title="Speed (°/s)", shareX=self.p1)
        self.p2.showGrid(x=True, y=True)

        self.curve1 = self.p1.plot(pen='y')
        self.curve2 = self.p2.plot(pen='c')

        # Downsample & clip for performance
        for c in (self.curve1, self.curve2):
            c.setDownsampling(auto=True, method='mean')
            c.setClipToView(True)

        # ─ Poll timer @ poll_hz ────────────────────────────────────────────────────
        interval_ms = int(1000 / poll_hz)
        self.data_timer = QtCore.QTimer(self)
        self.data_timer.timeout.connect(self.poll_and_update)
        self.data_timer.start(interval_ms)

        # ─ Command timer @ cmd_hz ──────────────────────────────────────────────────
        cmd_interval = int(1000 / cmd_hz)
        self.cmd_timer = QtCore.QTimer(self)
        self.cmd_timer.timeout.connect(self.send_speed_command)
        self.cmd_timer.start(cmd_interval)

    def on_speed_change(self, new_speed):
        """Update desired speed; commands will be sent at 10 Hz."""
        self.desired_speed = new_speed

    def send_speed_command(self):
        """Send the current desired speed + last angle at a fixed rate."""
        msg = compute(self.desired_speed, self.last_angle)
        # you can uncomment the next line to log every send
        # print("Sending speed cmd:", msg.strip())
        self.ser.write(msg.encode())

    def poll_and_update(self):
        """Read all available ASCII lines (~poll_hz×/s), parse, update buffers & UI."""
        now = time.time() - self.start_time

        # Drain serial buffer
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            m = ENC_PATTERN.match(line)
            if not m:
                continue

            angle = float(m.group(1))
            speed = float(m.group(2))

            # track last angle for future commands
            self.last_angle = angle

            # update device-speed label
            self.lbl_speed.setText(f"Latest device speed: {speed:.2f} °/s")

            # append to buffers
            self.angle_t.append(now)
            self.angle_d.append(angle)
            self.speed_t.append(now)
            self.speed_d.append(speed)

        # redraw curves
        if self.angle_t:
            self.curve1.setData(self.angle_t, self.angle_d)
        if self.speed_t:
            self.curve2.setData(self.speed_t, self.speed_d)

        # keep only last 1 s visible
        for p in (self.p1, self.p2):
            p.setXRange(now - 1.0, now)

    def closeEvent(self, ev):
        """Clean up serial on exit."""
        self.ser.close()
        ev.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = SerialPlotter(initial_speed=0.0, initial_angle=0.0)
    w.resize(800, 600)
    w.show()
    sys.exit(app.exec_())
