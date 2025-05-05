#!/usr/bin/env python3
"""Measure command‑ack latency between PC and STM32 motor controller.

Workflow
--------
1. Send a single speed command (#13:<speed_cm>:<angle>;;\r\n).
2. Record the exact wall‑clock time of the *write()*.
3. Wait for an **ACK line** from the STM32 firmware printed inside
   `CSpeedingMotor::CalculateSpeed`.
4. Report the latency (ACK_time − send_time).
5. Optionally continue listening for the first encoder feedback line to measure
   additional motor/encoder delay.

STM32 must print a line matching:
    "[ACK] speed_set = <float> cm/s"  (see firmware snippet below)
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path
import serial

# ────────────────────────────────────────────────────────────────────────────────
ACK_PATTERN = re.compile(r"\[ACK\]\s+speed_set\s*=\s*([-0-9.]+)")
ENC_PATTERN = re.compile(r"\[Encoder\]\s+angle\s*=\s*([-0-9.]+)°,\s*speed\s*=\s*([-0-9.]+)°/s")
MOTOR_ID = 13
DEFAULT_ANGLE = 0.0


def build_cmd(speed_cm_s: float, angle_deg: float = DEFAULT_ANGLE) -> bytes:
    return f"#{MOTOR_ID}:{speed_cm_s:.2f}:{angle_deg:.2f};;\r\n".encode()


# ────────────────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="Measure command→ACK latency")
    ap.add_argument("--speed", type=float, default=32, help="Speed to command [cm/s]")
    ap.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for ACK")
    ap.add_argument("--capture_encoder", action="store_true", help="Also measure time until first encoder feedback after ACK")
    args = ap.parse_args()

    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.1)
    time.sleep(0.1)

    # flush any old data
    ser.reset_input_buffer()

    cmd = build_cmd(args.speed)
    send_time = time.time()
    ser.write(cmd)
    print(f"Sent: {cmd.strip().decode()} @ {send_time:.6f}")

    ack_time: float | None = None
    enc_time: float | None = None

    deadline = send_time + args.timeout
    while time.time() < deadline and (ack_time is None or (args.capture_encoder and enc_time is None)):
        if ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if ack_time is None:
                m = ACK_PATTERN.match(line)
                if m:
                    ack_time = time.time()
                    print(f"ACK line received ({line}) @ {ack_time:.6f}")
                    continue
            if args.capture_encoder and ack_time is not None and enc_time is None:
                em = ENC_PATTERN.match(line)
                if em:
                    enc_time = time.time()
                    print(f"First encoder feedback @ {enc_time:.6f}")

    ser.close()

    if ack_time is None:
        print("ACK not received within timeout.")
        sys.exit(1)

    print(f"Latency (send → ACK): {(ack_time - send_time)*1e3:.2f} ms")

    if args.capture_encoder and enc_time is not None:
        print(f"Time ACK → first encoder feedback: {(enc_time - ack_time)*1e3:.2f} ms")
        print(f"Total send → encoder feedback    : {(enc_time - send_time)*1e3:.2f} ms")

    # send stop command
    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.1)
    ser.write(build_cmd(0.0))
    ser.close()


if __name__ == "__main__":
    main()
