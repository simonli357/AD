#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
import re
import csv
from pathlib import Path

import serial

# ────────────────────────────────────────────────────────────────────────────────
# ENC_PATTERN = re.compile(r"@5:([-0-9.]+);;")
# ENC_PATTERN = re.compile(r"@5:([-0-9.]+);([-0-9.]+);;")
ENC_PATTERN = re.compile(r"@5:([-0-9.]+);([-0-9.]+);([-0-9.]+);([-0-9.]+);;")


MOTOR_ID = 11
CM_TO_DEG = -146.0  # deg/s per cm/s (negative to fix sign)
BAND_TOL = 0.15     # ±10 % tolerance band for delay detection
HOLD_S   = 0.5      # must remain inside band for this long to count as settled

# ────────────────────────────────────────────────────────────────────────────────

def build_cmd(speed_cm_s: float, angle_deg: float) -> bytes:
    return f"#{MOTOR_ID}:{speed_cm_s:.4f}:{angle_deg:.2f};;\r\n".encode()

# ────────────────────────────────────────────────────────────────────────────────

def run_test(port: str, baud: int, speed_cm_s: float, duration: float, steer: float) -> tuple[list[float], list[float]]:
    ser = serial.Serial(port, baudrate=baud, timeout=0.1)
    time.sleep(0.1)

    t0 = time.time()
    next_send = 0.0
    last_angle = steer

    times: list[float] = []
    speeds_cm: list[float] = []
    cmd_speeds_cm: list[float] = []

    print(f"Running {duration}s at {speed_cm_s:.2f} cm/s …")
    try:
        while True:
            now = time.time()
            elapsed = now - t0

            if elapsed >= duration:
                break

            if elapsed >= next_send:
                ser.write(build_cmd(speed_cm_s, steer))
                next_send += 0.1

            while ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                m = ENC_PATTERN.match(line)
                if not m:
                    continue
                speed_cm = float(m.group(1))
                cmd_speed_cm = float(m.group(2))
                speeds_cm.append(speed_cm)
                cmd_speeds_cm.append(cmd_speed_cm)
                times.append(elapsed)

            if int(elapsed) % 1 == 0:
                sys.stdout.write(f"\r{elapsed:4.1f}s / {duration:.1f}s")
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nInterrupted by user — stopping motor.")
        ser.write(build_cmd(0.0, 0.0))  # stop motor
        time.sleep(0.1)

    finally:
        ser.write(build_cmd(0.0, 0.0))  # always stop motor on exit
        ser.close()
        print("\nSerial closed.")

    return times, speeds_cm, cmd_speeds_cm

# ────────────────────────────────────────────────────────────────────────────────

def estimate_delay(times: list[float], speeds: list[float], cmd_speed: float, tol: float = BAND_TOL, hold: float = HOLD_S) -> float:
    """Return first time |speed - cmd| ≤ tol·|cmd| for ≥ hold seconds."""
    import numpy as np

    t = np.array(times)
    v = np.array(speeds)
    if t.size == 0:
        return 0.0

    in_band = np.abs(v - cmd_speed) <= tol * abs(cmd_speed)
    if not in_band.any():
        return t[-1]

    idx = np.where(in_band)[0]
    segments = np.split(idx, np.where(np.diff(idx) != 1)[0] + 1)
    for seg in segments:
        if t[seg[-1]] - t[seg[0]] >= hold:
            return t[seg[0]]
    return t[-1]


def compute_stats(times: list[float], speeds: list[float], start_time: float) -> dict:
    import numpy as np

    t = np.array(times)
    s = np.array(speeds)
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

def save_csv(times: list[float], speeds: list[float], cmds: list[float], path: Path) -> None:
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "enc_speed_cm_s", "cmd_speed_cm_s"])
        writer.writerows(zip(times, speeds, cmds))
    print(f"CSV saved ➜ {path}")


def save_plot(times: list[float], speeds: list[float], cmd_speed: float, delay: float, stats: dict, path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot.")
        return

    plt.figure(figsize=(10, 5))
    plt.plot(times, speeds, label="Measured (cm/s)", color="blue")
    plt.axhline(cmd_speed, color="red", linestyle=":", label=f"Cmd {cmd_speed:.1f} cm/s")
    plt.axvline(delay, color="purple", linestyle="-.", label=f"Delay {delay:.2f}s")
    plt.axhline(stats["mean"], color="green", linestyle="--", label=f"Mean after delay {stats['mean']:.2f}")
    plt.fill_between(times, stats["mean"] - stats["std"], stats["mean"] + stats["std"], color="green", alpha=0.25, label=f"±1σ {stats['std']:.2f}")

    txt = (
        f"Delay: {delay:.2f} s\n"
        f"Mean: {stats['mean']:.2f} cm/s\n"
        f"σ noise: {stats['std']:.2f} cm/s\n"
        f"Min dev: {stats['min_dev']:.2f} cm/s\n"
        f"Max dev: {stats['max_dev']:.2f} cm/s"
    )
    plt.gca().text(0.02, 0.97, txt, transform=plt.gca().transAxes, va="top",
                   bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"))

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
    ap = argparse.ArgumentParser(description="Constant‑speed noise test with delay estimation (cm/s)")
    ap.add_argument("--cmd", type=float, default=50, help="Commanded speed [cm/s]")
    ap.add_argument("--steer", type=float, default=-10, help="Steering angle [deg]")
    ap.add_argument("--dur", type=float, default=5, help="Duration [s]")
    ap.add_argument("--csv", action="store_true", help="Save data to CSV file")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    times, speeds, cmd_speeds = run_test(args.port, args.baud, args.cmd, args.dur, args.steer)
    if len(speeds) < 3:
        print("Insufficient samples.")
        return

    delay = estimate_delay(times, speeds, args.cmd)
    stats = compute_stats(times, speeds, delay)

    print(f"Delay to ±10 % band   : {delay:.3f} s")
    print(f"Mean encoder speed    : {stats['mean']:.3f} cm/s")
    print(f"Noise σ               : {stats['std']:.3f} cm/s")
    print(f"Min deviation         : {stats['min_dev']:.3f} cm/s")
    print(f"Max deviation         : {stats['max_dev']:.3f} cm/s")

    script_dir = Path(__file__).parent
    if args.csv:
        save_csv(times, speeds, cmd_speeds, script_dir / "data" / f"encoder_noise_data_v{int(args.cmd)}_d{int(args.dur)}.csv")

    save_plot(times, speeds, args.cmd, delay, stats, script_dir / "plots" / f"encoder_noise_plot_v{int(args.cmd)}_d{int(args.dur)}.png")

    # stop motor
    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.1)
    ser.write(build_cmd(0.0, 0.0))
    ser.close()


if __name__ == "__main__":
    main()

