#!/usr/bin/env python3
"""Encoder linear‑speed sweep (cm/s) with CSV logging

Changes from previous version
=============================
1. **Sign fix** – encoder reports the opposite sign, so measured speed is now
   multiplied by **-1** before conversion to cm/s.
2. **CSV export** – after the run finishes two files are written:

   * `cmd_log.csv`   → `time_s, cmd_cm_s`
   * `meas_log.csv`  → `time_s, meas_cm_s`

   Those can be imported directly into Excel, MATLAB, Python pandas, etc.
3. Plot export is still saved as `speed_test.png`.
"""

import sys
import time
import re
import csv
from collections import deque
from typing import Sequence

import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter
from pathlib import Path

CURRENT_DIR = Path(__file__).parent
# ────────────────────────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyACM0"  # <- adjust for your system
BAUD_RATE   = 115200
TIMEOUT_S   = 0.1

CM_TO_DEG = 146.0  # deg/s per cm/s (for measurement conversion only)

# 10‑step profile (cm/s) – keep |v| <= 50
# SPEED_SEQUENCE_CM = [-50, -40, -30, -20, -10, 0, 10, 20, 30, 50]
SPEED_SEQUENCE_CM = [46, -45, 0, 40, -30, 30, -20, 20, -10, 10]
# SPEED_SEQUENCE_CM = [-10, 0, 10, -20, 20, -30, 30, -40, 40, 50]

HOLD_TIME_S       = 60.0

# ENC_PATTERN = re.compile(r"\[Encoder\]\s+angle\s*=\s*([-0-9.]+)°,\s*speed\s*=\s*([-0-9.]+)°/s")
ENC_PATTERN = re.compile(r"@5:([-0-9.]+);([-0-9.]+);;")


MOTOR_ID = 13

def build_cmd(speed_cm_s: float, angle_deg: float) -> bytes:
    """Return ASCII command. Device expects cm/s directly."""
    return f"#{MOTOR_ID}:{speed_cm_s:.2f}:{angle_deg:.2f};;\r\n".encode()

# ────────────────────────────────────────────────────────────────────────────────
class SpeedSweep(QtWidgets.QWidget):
    """Runs the sweep, plots in real‑time, and logs to CSV."""

    def __init__(
        self,
        port: str = SERIAL_PORT,
        baud: int = BAUD_RATE,
        timeout: float = TIMEOUT_S,
        speeds_cm: Sequence[float] = SPEED_SEQUENCE_CM,
        hold_s: float = HOLD_TIME_S,
        poll_hz: int = 30,
        send_hz: int = 20,
        bufsize: int = 15_000,
    ) -> None:
        super().__init__()
        self.setWindowTitle("Encoder Speed‑Sweep (cm/s) – CSV Logging")

        # Serial setup
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        time.sleep(0.1)
        self.t0 = time.time()

        # Test schedule
        self.speeds_cm = list(speeds_cm)
        if len(self.speeds_cm) != 10:
            raise ValueError("Expected exactly ten speeds (SPEED_SEQUENCE_CM).")
        self.hold_s = hold_s
        self.idx = 0
        self.desired_cm_s = self.speeds_cm[0]

        # Last absolute angle for command context
        self.last_angle_deg = 0.0

        # Data buffers
        self.t_cmd,  self.cmd_cm  = deque(maxlen=bufsize), deque(maxlen=bufsize)
        self.t_meas, self.meas_cm = deque(maxlen=bufsize), deque(maxlen=bufsize)

        # UI setup
        vbox = QtWidgets.QVBoxLayout(self)
        self.lbl_status = QtWidgets.QLabel()
        vbox.addWidget(self.lbl_status)

        self.pg_widget = pg.GraphicsLayoutWidget()
        vbox.addWidget(self.pg_widget)
        self.plot = self.pg_widget.addPlot(title="Speed (cm/s)")
        self.plot.showGrid(x=True, y=True)
        self.curve_cmd  = self.plot.plot(pen="m", name="Command")
        self.curve_meas = self.plot.plot(pen="c", name="Measured")
        self.plot.addLegend(offset=(10, 10))
        for c in (self.curve_cmd, self.curve_meas):
            c.setDownsampling(auto=True, method="mean")
            c.setClipToView(True)

        # Timers
        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.poll_serial)
        self.poll_timer.start(int(1000 / poll_hz))

        self.send_timer = QtCore.QTimer(self)
        self.send_timer.timeout.connect(self.send_command)
        self.send_timer.start(int(1000 / send_hz))

        self.step_timer = QtCore.QTimer(self)
        self.step_timer.timeout.connect(self.advance_speed)
        self.step_timer.start(int(hold_s * 1000))

        # Initial command
        self.lbl_status.setText(f"Step 1/10 → {self.desired_cm_s:.1f} cm/s")
        self.ser.write(build_cmd(self.desired_cm_s, self.last_angle_deg))

    # ───────── timers ─────────
    def advance_speed(self) -> None:
        self.idx += 1
        if self.idx >= len(self.speeds_cm):
            self.finish()
            return
        self.desired_cm_s = self.speeds_cm[self.idx]
        self.lbl_status.setText(f"Step {self.idx+1}/10 → {self.desired_cm_s:.1f} cm/s")

    def send_command(self) -> None:
        now = time.time() - self.t0
        self.ser.write(build_cmd(self.desired_cm_s, self.last_angle_deg))
        self.t_cmd.append(now)
        self.cmd_cm.append(self.desired_cm_s)

    def poll_serial(self) -> None:
        now = time.time() - self.t0
        while self.ser.in_waiting:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            m = ENC_PATTERN.match(line)
            if not m:
                continue
            # angle_deg = float(m.group(1))
            # speed_deg_s_raw = float(m.group(2))
            # self.last_angle_deg = angle_deg
            # speed_cm_s = (-speed_deg_s_raw) / CM_TO_DEG
            angle_deg = float(m.group(1))
            speed_cm_s = float(m.group(2))
            self.last_angle_deg = angle_deg

            self.t_meas.append(now)
            self.meas_cm.append(speed_cm_s)

        # Update curves
        if self.t_cmd:
            self.curve_cmd.setData(self.t_cmd, self.cmd_cm)
        if self.t_meas:
            self.curve_meas.setData(self.t_meas, self.meas_cm)
        span = len(self.speeds_cm) * self.hold_s * 1.2
        self.plot.setXRange(now - span, now)

    # ───────── finish & export ─────────
    def finish(self) -> None:
        # Stop timers
        for t in (self.poll_timer, self.send_timer, self.step_timer):
            t.stop()

        self.lbl_status.setText("Run complete – saving data …")
        self.export_plot()
        self.export_csv()
        self.lbl_status.setText("Run complete – data saved (PNG + CSV)")

    def export_plot(self, filename: str = "speed_test.png") -> None:
        filename = CURRENT_DIR / filename
        exporter = ImageExporter(self.plot)
        exporter.parameters()["width"] = 1600
        exporter.export(str(filename))  # Convert Path to str here
        print(f"Saved plot to {filename}")

    def export_csv(self) -> None:
        # Command log
        filename = CURRENT_DIR / "cmd_log.csv"
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "cmd_cm_s"])
            writer.writerows(zip(self.t_cmd, self.cmd_cm))
        # Measurement log
        meas_filename = CURRENT_DIR / "meas_log.csv"
        with open(meas_filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "meas_cm_s"])
            writer.writerows(zip(self.t_meas, self.meas_cm))
        print("Saved cmd_log.csv and meas_log.csv")

    # ───────── cleanup ─────────
    def closeEvent(self, ev):
        try:
            # send a speed of 0 to stop the motor
            self.ser.write(build_cmd(0.0, self.last_angle_deg))
            # Close serial port
            self.ser.close()
        finally:
            ev.accept()

# ────────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = SpeedSweep()
    w.resize(900, 600)
    w.show()
    sys.exit(app.exec_())