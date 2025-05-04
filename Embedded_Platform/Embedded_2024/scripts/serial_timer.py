#!/usr/bin/env python3
import serial
import time

# ── Configuration ────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200
TIMEOUT     = 0.1      # seconds for readline
DURATION    = 5.0      # total sample time in seconds

# ── Open & prime the port ────────────────────────────────────────
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
time.sleep(0.1)
ser.reset_input_buffer()

# ── Collect arrival timestamps ───────────────────────────────────
timestamps = []
t0 = time.time()
while time.time() - t0 < DURATION:
    line = ser.readline()
    if not line:
        continue
    # record arrival time immediately when we get a line
    timestamps.append(time.time())

# ── Compute stats ────────────────────────────────────────────────
n = len(timestamps)
if n < 2:
    print("Not enough samples collected.")
else:
    # build list of inter-arrival times (in seconds)
    deltas = [timestamps[i] - timestamps[i-1] for i in range(1, n)]
    avg_dt  = sum(deltas) / len(deltas)
    min_dt  = min(deltas)
    max_dt  = max(deltas)
    freq_hz = 1.0 / avg_dt if avg_dt > 0 else float('inf')

    print(f"Samples: {n} over {DURATION:.1f}s")
    print(f"Period: avg {avg_dt*1e3:.3f} ms, min {min_dt*1e3:.3f} ms, max {max_dt*1e3:.3f} ms")
    print(f"Rate: {freq_hz:.1f} Hz")
