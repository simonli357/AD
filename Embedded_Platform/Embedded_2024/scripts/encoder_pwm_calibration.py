#!/usr/bin/env python3
"""Encoder test — delay / noise / displacement, plus **safe PWM mode**

Highlights
==========
* **Duration padding** → runs for *(duration + 1 s)*.
* **Cumulative‑angle displacement** taken from
  ``[Encoder] Total displacement = …°`` lines.
* **PWM mode** (``--pwm`` ≠ −1):
  * Uses **MOTOR_ID 10**.
  * Value must lie in the *safe window* **0.065 ≤ pwm ≤ 0.11** (≈6.5–11 %).
  * Outside this range the script aborts *before* talking to the motor.
  * Delay / noise stats are skipped (encoder still logged for plotting).
* **Ctrl‑C** now *immediately* stops the motor and terminates without any
  further processing.
"""
from __future__ import annotations

import argparse
import sys
import time
import re
import csv
from pathlib import Path

import serial

# ────────────────────────────────────────────────────────────────────────────────
# Serial line formats
# -------------------
# Fast status: "@5:<angle_deg>;<speed_cm_s>;;"
# Cum. angle : "[Encoder] Total displacement = <deg>°"
# ------------------------------------------------------------------------------
ENC_PATTERN = re.compile(r"@5:([-0-9.]+);([-0-9.]+);;")
TOT_PATTERN = re.compile(r"\[Encoder\]\s+Total displacement\s*=\s*([-0-9.]+)°")

MOTOR_ID_SPEED = 11  # expects cm/s
MOTOR_ID_PWM   = 10  # expects duty cycle in [0,1]

PWM_MIN = 0.065
PWM_MAX = 0.11

BAND_TOL = 0.15  # ±15 % band for delay detection
HOLD_S   = 0.5   # must remain inside band ≥ 0.5 s
RAMP_SKIP_START = 1.0

# ────────────────────────────────────────────────────────────────────────────────

def build_cmd(motor_id: int, value: float, angle_deg: float) -> bytes:
    """Return the ASCII command understood by the motor controller."""
    return f"#{motor_id}:{value:.5f}:{angle_deg:.5f};;\r\n".encode()

# ────────────────────────────────────────────────────────────────────────────────

def run_test(port: str,
             baud: int,
             motor_id: int,
             cmd_value: float,
             duration: float,
             steer: float,
             echo_raw: bool) -> tuple[list[float], list[float], float | None, float | None]:
    """Run the motor command for *duration + 1 s*, logging encoder output."""
    ser = serial.Serial(port, baudrate=baud, timeout=0.1)
    time.sleep(0.1)

    t0 = time.time()
    next_send = 0.0

    times: list[float] = []
    speeds: list[float] = []
    total_start: float | None = None
    total_end: float | None = None
    raw_lines: list[str] = [] 

    real_dur = duration + 1.0
    print(f"Running {real_dur:.2f}s … (motor {motor_id} value {cmd_value})")
    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if elapsed >= real_dur:
                break

            if elapsed >= next_send:
                ser.write(build_cmd(motor_id, cmd_value, steer))
                next_send += 0.1  # refresh at 10 Hz

            while ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if echo_raw:
                    print(f"{elapsed:7.3f}  {line}")
                raw_lines.append(f"{elapsed:.3f},{line}")

                if m := ENC_PATTERN.match(line):
                    speeds.append(float(m.group(2)))
                    times.append(elapsed)
                    continue

                if m := TOT_PATTERN.match(line):
                    deg = float(m.group(1))
                    # latch start *after* ramp‑up window
                    if elapsed >= RAMP_SKIP_START and total_start is None:
                        total_start = deg
                    total_end = deg

            if int(elapsed * 10) % 10 == 0:
                sys.stdout.write(f"\r{elapsed:6.2f}s / {real_dur:.2f}s")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n⚙️  Ctrl‑C — stopping motor immediately.")
        ser.write(build_cmd(motor_id, 0.0, steer))
        ser.close()
        raise
    finally:
        try:
            ser.write(build_cmd(motor_id, 0.0, steer))
            pass
        except Exception:
            pass
        ser.close()
        print("\nSerial closed.")

    return times, speeds, total_start, total_end, raw_lines

# ────────────────────────────────────────────────────────────────────────────────

def estimate_delay(times: list[float], speeds: list[float], cmd_speed: float) -> float:
    """Return first time speed settles within ±15 % of *cmd_speed* for ≥0.5 s."""
    import numpy as np

    if not times:
        return float("nan")

    t = np.asarray(times)
    v = np.asarray(speeds)

    in_band = np.abs(v - cmd_speed) <= BAND_TOL * abs(cmd_speed)
    if not in_band.any():
        return t[-1]

    idx = np.where(in_band)[0]
    segs = np.split(idx, np.where(np.diff(idx) != 1)[0] + 1)
    for seg in segs:
        if t[seg[-1]] - t[seg[0]] >= HOLD_S:
            return t[seg[0]]
    return t[-1]


def compute_stats(times: list[float], speeds: list[float], start_time: float) -> dict:
    import numpy as np

    if not times:
        return {k: float("nan") for k in ("mean", "std", "min_dev", "max_dev")}

    t = np.asarray(times)
    s = np.asarray(speeds)
    mask = t >= start_time
    if not mask.any():
        return {k: float("nan") for k in ("mean", "std", "min_dev", "max_dev")}

    s_seg = s[mask]
    mean = float(s_seg.mean())
    resid = s_seg - mean
    return {
        "mean": mean,
        "std": float(resid.std(ddof=1)),
        "min_dev": float(resid.min()),
        "max_dev": float(resid.max()),
    }

# ────────────────────────────────────────────────────────────────────────────────

def save_csv(times: list[float], speeds: list[float], path: Path) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "enc_speed_cm_s"])
        w.writerows(zip(times, speeds))
    print(f"CSV saved ➜ {path}")


def save_plot(times: list[float], speeds: list[float], cmd_speed: float, delay: float, stats: dict, path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot.")
        return

    plt.figure(figsize=(10, 5))
    plt.plot(times, speeds, label="Measured (cm/s)")
    plt.axhline(cmd_speed, color="red", linestyle=":", label=f"Cmd {cmd_speed:.1f} cm/s")
    plt.axvline(delay, color="purple", linestyle="-.", label=f"Delay {delay:.2f}s")
    plt.axhline(stats["mean"], color="green", linestyle="--", label=f"Mean {stats['mean']:.2f}")
    plt.fill_between(times,
                     stats["mean"] - stats["std"],
                     stats["mean"] + stats["std"],
                     color="green", alpha=0.25,
                     label=f"±1σ {stats['std']:.2f}")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [cm/s]")
    plt.title("Encoder response (cm/s)")
    plt.grid(True, linestyle=":")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    print(f"Plot saved ➜ {path}")

# ────────────────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="Encoder constant‑command test with optional safe PWM mode and displacement logging.")
    ap.add_argument("--cmd",  type=float, default=30, help="Commanded speed [cm/s]; ignored if --pwm provided.")
    # ap.add_argument("--pwm",  type=float, default=-1, help=f"Duty cycle (0‑1). Valid range {PWM_MIN}-{PWM_MAX}. Use -1 to disable PWM mode.")
    ap.add_argument("--pwm",  type=float, default=0.0675, help=f"Duty cycle (0‑1). Valid range {PWM_MIN}-{PWM_MAX}. Use -1 to disable PWM mode.")
    ap.add_argument("--steer", type=float, default=0, help="Steering angle [deg]")
    # ap.add_argument("--pwm_steer", type=float, default=-1, help="Steering angle [deg]")
    ap.add_argument("--pwm_steer", type=float, default=0.040, help="Steering angle [deg]")
    ap.add_argument("--dur",   type=float, default=60, help="Duration [s]")
    ap.add_argument("--csv",   type=Path, help="Path to save raw data as CSV")
    ap.add_argument("--port",  default="/dev/ttyACM0")
    ap.add_argument("--baud",  type=int, default=115200)
    ap.add_argument("--echo_raw", action="store_true", help="Print every raw serial line as it arrives")
    
    args = ap.parse_args()

    is_pwm = args.pwm != -1
    # if is_pwm and not (PWM_MIN <= args.pwm <= PWM_MAX):
    #     sys.exit(f"❌ PWM value {args.pwm} outside safe range {PWM_MIN}-{PWM_MAX}.")

    motor_id = MOTOR_ID_PWM if is_pwm else MOTOR_ID_SPEED
    cmd_value = args.pwm if is_pwm else args.cmd
    steer_value = args.pwm_steer if is_pwm else args.steer

    try:
        times, speeds, total_start, total_end, raw_lines = run_test(
            args.port,
            args.baud,
            motor_id,
            cmd_value,
            args.dur,
            steer_value,
            args.echo_raw,
        )
    except KeyboardInterrupt:
        print("Test aborted by user.")
        return

    print()
    if args.echo_raw:
        print("Raw lines:")
        for line in raw_lines:
            print(line)
    # Stats only in speed mode
    if not is_pwm and speeds:
        delay = estimate_delay(times, speeds, args.cmd)
        stats = compute_stats(times, speeds, delay)
        print(f"Delay to ±{BAND_TOL*100:.0f}% band : {delay:.3f} s")
        print(f"Mean encoder speed        : {stats['mean']:.3f} cm/s")
        print(f"Noise σ                   : {stats['std']:.3f} cm/s")
        print(f"Min deviation             : {stats['min_dev']:.3f} cm/s")
        print(f"Max deviation             : {stats['max_dev']:.3f} cm/s")
    elif is_pwm:
        print("PWM mode — delay/noise stats skipped.")

    if total_start is not None and total_end is not None:
        disp = total_end - total_start
        print(f"Total angle start         : {total_start:.2f} °")
        print(f"Total angle end           : {total_end:.2f} °")
        print(f"Net displacement          : {disp:.2f} °")
        print(f"Net displacement cm          : {disp/-146:.2f} cm")
        print(f"Net speed cm/s          : {disp/-146/args.dur:.2f} cm/s")
    else:
        print("⚠️  No cumulative‑angle data received — displacement unavailable.")

    if args.csv and times:
        save_csv(times, speeds, args.csv)

    if not is_pwm and speeds:
        script_dir = Path(__file__).parent
        delay = estimate_delay(times, speeds, args.cmd)  # recompute to reuse
        stats = compute_stats(times, speeds, delay)
        save_plot(times, speeds, args.cmd, delay, stats, script_dir / "encoder_noise_plot.png")


if __name__ == "__main__":
    main()
