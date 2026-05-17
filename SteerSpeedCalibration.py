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
ENC_PATTERN = re.compile(r"@5:([-0-9.]+);([-0-9.]+);([-0-9.]+);([-0-9.]+);;")
IMU_PATTERN = re.compile(r"@7:([-0-9.]+);([-0-9.]+);\s*(\d+);\s*(\d+);\s*(\d+);\s*(\d+);;")

MOTOR_ID = 10
CM_TO_DEG = 140.63  # deg/s per cm/s (negative to fix sign)
BAND_TOL = 0.15     # ±10 % tolerance band for delay detection
HOLD_S   = 0.5      # must remain inside band for this long to count as settled

ZERO_STEER_PWM = 0.0742
ZERO_SPEED_PWM = 0.074568

# ────────────────────────────────────────────────────────────────────────────────

def build_cmd(speed_pwm: float, angle_pwm: float) -> bytes:
    return f"#{MOTOR_ID}:{speed_pwm:.4f}:{angle_pwm:.4f};;\r\n".encode()

# ────────────────────────────────────────────────────────────────────────────────

def run_test(port: str, baud: int, speed_pwm: float, duration: float, steer: float):
    ser = serial.Serial(port, baudrate=baud, timeout=0.1)
    time.sleep(0.1)

    t0 = time.time()
    next_send = 0.0

    times: list[float] = []
    speeds_cm: list[float] = []
    cmd_speeds_cm: list[float] = []

    imu_times: list[float] = []
    yaws: list[float] = []

    print(f"Running {duration}s at {speed_pwm:.4f} PWM …")
    try:
        while True:
            now = time.time()
            elapsed = now - t0

            if elapsed >= duration:
                break

            if elapsed >= next_send:
                ser.write(build_cmd(speed_pwm, steer))
                next_send += 0.1

            while ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()

                m = ENC_PATTERN.match(line)
                if m:
                    speeds_cm.append(float(m.group(1)))
                    cmd_speeds_cm.append(float(m.group(2)))
                    times.append(elapsed)
                    continue

                m = IMU_PATTERN.match(line)
                if m:
                    yaws.append(float(m.group(2)))
                    imu_times.append(elapsed)

            if int(elapsed) % 1 == 0:
                sys.stdout.write(f"\r{elapsed:4.1f}s / {duration:.1f}s")
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nInterrupted by user — stopping motor.")
        ser.write(build_cmd(ZERO_SPEED_PWM, steer))
        time.sleep(0.1)

    finally:
        ser.write(build_cmd(ZERO_SPEED_PWM, steer))
        ser.close()
        print("\nSerial closed.")

    return times, speeds_cm, cmd_speeds_cm, imu_times, yaws

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
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "enc_speed_cm_s", "cmd_speed_cm_s"])
        writer.writerows(zip(times, speeds, cmds))
    print(f"CSV saved ➜ {path}")


def save_imu_csv(imu_times: list[float], yaws: list[float], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "yaw_deg"])
        writer.writerows(zip(imu_times, yaws))
    print(f"IMU CSV saved ➜ {path}")


def save_plot(times: list[float], speeds: list[float], cmd_speed: float, delay: float, stats: dict, path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot.")
        return

    path.parent.mkdir(parents=True, exist_ok=True)
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


def save_imu_plot(imu_times: list[float], yaws: list[float], path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping IMU plot.")
        return

    if not imu_times:
        print("No IMU data recorded; skipping IMU plot.")
        return

    import numpy as np

    path.parent.mkdir(parents=True, exist_ok=True)

    # Insert NaN where consecutive yaw values jump by more than 180° (wrap-around)
    t_arr = np.array(imu_times, dtype=float)
    y_arr = np.array(yaws, dtype=float)
    breaks = np.where(np.abs(np.diff(y_arr)) > 180)[0] + 1
    t_plot = np.insert(t_arr, breaks, np.nan)
    y_plot = np.insert(y_arr, breaks, np.nan)

    plt.figure(figsize=(10, 4))
    plt.plot(t_plot, y_plot, label="Yaw (deg)", color="darkorange")
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw [deg]")
    plt.title("IMU Yaw vs Time")
    plt.grid(True, linestyle=":")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    print(f"IMU plot saved ➜ {path}")


# ────────────────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="Constant‑speed noise test with IMU yaw recording (cm/s)")
    ap.add_argument("--cmd", type=float, default=ZERO_SPEED_PWM, help="Commanded speed [Duty Cycle %]")
    ap.add_argument("--steer", type=float, default=ZERO_STEER_PWM, help="Steering angle [Duty Cycle %]")
    ap.add_argument("--dur", type=float, default=5, help="Duration [s]")
    ap.add_argument("--csv", action="store_true", help="Save data to CSV files")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    times, speeds, cmd_speeds, imu_times, yaws = run_test(args.port, args.baud, args.cmd, args.dur, args.steer)

    script_dir = Path(__file__).parent
    tag = f"v{int(args.cmd)}_d{int(args.dur)}"

    if len(speeds) >= 3:
        delay = estimate_delay(times, speeds, args.cmd)
        stats = compute_stats(times, speeds, delay)

        print(f"Delay to ±10 % band   : {delay:.3f} s")
        print(f"Mean encoder speed    : {stats['mean']:.3f} cm/s")
        print(f"Noise σ               : {stats['std']:.3f} cm/s")
        print(f"Min deviation         : {stats['min_dev']:.3f} cm/s")
        print(f"Max deviation         : {stats['max_dev']:.3f} cm/s")

        if args.csv:
            save_csv(times, speeds, cmd_speeds, script_dir / "data" / f"encoder_noise_data_{tag}.csv")
        #save_plot(times, speeds, args.cmd, delay, stats, script_dir / "plots" / f"encoder_noise_plot_{tag}.png")
    else:
        print("Insufficient encoder samples.")

    if args.csv:
        save_imu_csv(imu_times, yaws, script_dir / "data" / f"imu_yaw_{tag}.csv")
    #save_imu_plot(imu_times, yaws, script_dir / "plots" / f"imu_yaw_{tag}.png")


if __name__ == "__main__":
    main()
