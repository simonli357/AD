#!/usr/bin/env python3
"""Reduced car function test.

Only runs:
    1.1  Communication check — listen for encoder/IMU telemetry
    1.4  Steering sweep at slow forward speed: 0 → max-left → max-right → 0

Protocol matches encoder_imu_test_cmd.py (cm/s, deg).
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import serial

# ────────────────────────────────────────────────────────────────────────────────
ENC_PATTERN = re.compile(r"@5:([-0-9.]+);([-0-9.]+);([-0-9.]+);([-0-9.]+);;")
IMU_PATTERN = re.compile(r"@7:([-0-9.]+);([-0-9.]+);\s*(\d+);\s*(\d+);\s*(\d+);\s*(\d+);;")

MOTOR_ID = 11
CMD_PERIOD_S = 0.1            # 10 Hz command rate
DEFAULT_MAX_STEER_DEG = 20.0
SLOW_FORWARD_CM_S = 10.0       # slow forward speed during steering sweep


# ────────────────────────────────────────────────────────────────────────────────
def build_cmd(speed_cm_s: float, angle_deg: float) -> bytes:
    return f"#{MOTOR_ID}:{speed_cm_s:.4f}:{angle_deg:.2f};;\r\n".encode()


@dataclass
class Sample:
    t: float
    cmd_speed: float
    cmd_steer: float
    enc_speed: float = float("nan")
    enc_cmd_speed: float = float("nan")
    yaw: float = float("nan")


@dataclass
class TestRecord:
    name: str
    samples: list[Sample] = field(default_factory=list)
    user_pass: bool | None = None
    user_note: str = ""


# ────────────────────────────────────────────────────────────────────────────────
class CarLink:
    """Thin wrapper around serial that drains and parses telemetry."""

    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.last_enc_speed: float = float("nan")
        self.last_enc_cmd: float = float("nan")
        self.last_yaw: float = float("nan")
        self.enc_seen = False
        self.imu_seen = False

    def send(self, speed_cm_s: float, angle_deg: float) -> None:
        self.ser.write(build_cmd(speed_cm_s, angle_deg))

    def drain(self) -> None:
        while self.ser.in_waiting:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            m = ENC_PATTERN.match(line)
            if m:
                self.last_enc_speed = float(m.group(1))
                self.last_enc_cmd = float(m.group(2))
                self.enc_seen = True
                continue
            m = IMU_PATTERN.match(line)
            if m:
                self.last_yaw = float(m.group(2))
                self.imu_seen = True

    def stop(self) -> None:
        try:
            self.send(0.0, 0.0)
            time.sleep(0.05)
            self.send(0.0, 0.0)
        except Exception:
            pass

    def close(self) -> None:
        self.stop()
        self.ser.close()


# ────────────────────────────────────────────────────────────────────────────────
def run_segment(
    link: CarLink,
    record: TestRecord,
    duration_s: float,
    speed_fn,
    steer_fn,
    *,
    label: str = "",
) -> bool:
    t0 = time.time()
    next_send = 0.0
    while True:
        now = time.time()
        elapsed = now - t0
        if elapsed >= duration_s:
            break

        cmd_v = float(speed_fn(elapsed))
        cmd_s = float(steer_fn(elapsed))

        if elapsed >= next_send:
            link.send(cmd_v, cmd_s)
            next_send += CMD_PERIOD_S

        link.drain()
        record.samples.append(
            Sample(
                t=elapsed,
                cmd_speed=cmd_v,
                cmd_steer=cmd_s,
                enc_speed=link.last_enc_speed,
                enc_cmd_speed=link.last_enc_cmd,
                yaw=link.last_yaw,
            )
        )

        if label and int(elapsed * 2) != int((elapsed - 0.02) * 2):
            sys.stdout.write(f"\r  {label}: {elapsed:5.2f}/{duration_s:.2f}s  cmd={cmd_v:+6.2f} cm/s  steer={cmd_s:+6.2f}°  enc={link.last_enc_speed:+7.2f}")
            sys.stdout.flush()

        time.sleep(0.005)

    if label:
        sys.stdout.write("\n")
    return True


def hold(link: CarLink, record: TestRecord, speed: float, steer: float, seconds: float, label: str = "") -> bool:
    return run_segment(link, record, seconds, lambda t: speed, lambda t: steer, label=label)


def steer_ramp(link: CarLink, record: TestRecord, speed: float, s0: float, s1: float, seconds: float, label: str = "") -> bool:
    return run_segment(
        link,
        record,
        seconds,
        lambda t: speed,
        lambda t, s0=s0, s1=s1, T=seconds: s0 + (s1 - s0) * (t / T),
        label=label,
    )


# ────────────────────────────────────────────────────────────────────────────────
def yes_no(prompt: str, default_yes: bool = True) -> bool:
    suffix = "[Y/n]" if default_yes else "[y/N]"
    while True:
        ans = input(f"{prompt} {suffix} ").strip().lower()
        if not ans:
            return default_yes
        if ans in ("y", "yes"):
            return True
        if ans in ("n", "no"):
            return False


def confirm_pass(record: TestRecord) -> None:
    ok = yes_no(f"  → Did '{record.name}' behave correctly?", default_yes=True)
    record.user_pass = ok
    record.user_note = input("  → Optional note (press Enter to skip): ").strip()


def wait_enter(prompt: str) -> None:
    input(f"{prompt} (press Enter to continue) ")


# ────────────────────────────────────────────────────────────────────────────────
def save_csv(record: TestRecord, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "cmd_speed_cm_s", "cmd_steer_deg", "enc_speed_cm_s", "enc_cmd_speed_cm_s", "yaw_deg"])
        for s in record.samples:
            w.writerow([f"{s.t:.4f}", s.cmd_speed, s.cmd_steer, s.enc_speed, s.enc_cmd_speed, s.yaw])


def save_plot(record: TestRecord, path: Path, *, plot_steer: bool = False, plot_yaw: bool = False) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib/numpy not installed; skipping plot.")
        return
    if not record.samples:
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    t = np.array([s.t for s in record.samples])
    cmd_v = np.array([s.cmd_speed for s in record.samples])
    enc_v = np.array([s.enc_speed for s in record.samples])
    cmd_s = np.array([s.cmd_steer for s in record.samples])
    yaw = np.array([s.yaw for s in record.samples])

    n_panels = 1 + int(plot_steer) + int(plot_yaw)
    fig, axes = plt.subplots(n_panels, 1, figsize=(10, 3.2 * n_panels), sharex=True)
    if n_panels == 1:
        axes = [axes]

    ax = axes[0]
    ax.plot(t, cmd_v, "r:", label="cmd speed")
    ax.plot(t, enc_v, "b-", label="encoder speed")
    ax.set_ylabel("Speed [cm/s]")
    ax.set_title(record.name)
    ax.grid(True, linestyle=":")
    ax.legend(loc="best")

    idx = 1
    if plot_steer:
        ax = axes[idx]
        ax.plot(t, cmd_s, "g-", label="cmd steering")
        ax.set_ylabel("Steer [deg]")
        ax.grid(True, linestyle=":")
        ax.legend(loc="best")
        idx += 1
    if plot_yaw:
        ax = axes[idx]
        breaks = np.where(np.abs(np.diff(yaw)) > 180)[0] + 1
        t_p = np.insert(t, breaks, np.nan)
        y_p = np.insert(yaw, breaks, np.nan)
        ax.plot(t_p, y_p, color="darkorange", label="yaw")
        ax.set_ylabel("Yaw [deg]")
        ax.grid(True, linestyle=":")
        ax.legend(loc="best")

    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close(fig)


def write_summary(records: list[TestRecord], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write(f"Car function test (steer-only) — {datetime.now().isoformat(timespec='seconds')}\n")
        f.write("=" * 60 + "\n")
        for r in records:
            status = "PASS" if r.user_pass else ("FAIL" if r.user_pass is False else "SKIP")
            f.write(f"[{status}] {r.name}\n")
            if r.user_note:
                f.write(f"       note: {r.user_note}\n")


# ────────────────────────────────────────────────────────────────────────────────
def test_combined(link: CarLink, max_steer: float, speed: float) -> list[TestRecord]:
    """Comm check + steering sweep in one pass.

    The sweep itself drains encoder/IMU telemetry continuously, so the
    communication check is satisfied implicitly — we just inspect the
    enc_seen/imu_seen flags afterwards.
    """
    comm_rec = TestRecord(name="1.1_communication")
    sweep_rec = TestRecord(name="1.4_steering_sweep")
    print(f"\n=== Combined test 1.1 + 1.4 — comm check + steering sweep at {speed:.1f} cm/s, ±{max_steer:.1f}° ===")
    if not yes_no("Is the car on the box with wheels free (or otherwise safe to drive slowly)?", True):
        for rec in (comm_rec, sweep_rec):
            rec.user_note = "user aborted"
            rec.user_pass = False
        return [comm_rec, sweep_rec]

    sweep_t = 4.0
    # 0 → -max (left)
    steer_ramp(link, sweep_rec, speed, 0.0, -max_steer, sweep_t, label="→ left")
    hold(link, sweep_rec, speed, -max_steer, 1.5, label="hold left")
    # -max → +max
    steer_ramp(link, sweep_rec, speed, -max_steer, +max_steer, 2 * sweep_t, label="left → right")
    hold(link, sweep_rec, speed, +max_steer, 1.5, label="hold right")
    # +max → 0
    steer_ramp(link, sweep_rec, speed, +max_steer, 0.0, sweep_t, label="→ centre")
    link.stop()

    print(f"  Encoder messages seen : {'YES' if link.enc_seen else 'NO'}")
    print(f"  IMU messages seen     : {'YES' if link.imu_seen else 'NO'}")
    if not link.enc_seen:
        print("  WARNING: no encoder telemetry — sweep data is uninformative.")

    # Auto-pass the comm test on telemetry, mirror the user's verdict on the sweep.
    comm_rec.user_pass = link.enc_seen and link.imu_seen
    comm_rec.user_note = "auto: telemetry seen during sweep" if comm_rec.user_pass else "auto: missing telemetry"
    confirm_pass(sweep_rec)
    return [comm_rec, sweep_rec]


# ────────────────────────────────────────────────────────────────────────────────
def main() -> None:
    ap = argparse.ArgumentParser(description="Reduced car function test (comm check + slow steering sweep)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=float, default=SLOW_FORWARD_CM_S, help="Slow forward speed during sweep [cm/s]")
    ap.add_argument("--max-steer", type=float, default=DEFAULT_MAX_STEER_DEG, help="Max steering angle [deg]")
    ap.add_argument("--out-root", default=None, help="Root folder for results (default: ./car_function_results)")
    args = ap.parse_args()

    script_dir = Path(__file__).parent
    out_root = Path(args.out_root) if args.out_root else script_dir / "car_function_results"
    run_dir = out_root / datetime.now().strftime("steer_only_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"Results will be saved to: {run_dir}")

    link = CarLink(args.port, args.baud)
    records: list[TestRecord] = []

    try:
        records.extend(test_combined(link, args.max_steer, args.speed))
    except KeyboardInterrupt:
        print("\n!! Interrupted — stopping motor.")
    finally:
        link.close()

    print("\nSaving CSVs and plots…")
    for rec in records:
        if not rec.samples:
            continue
        save_csv(rec, run_dir / "data" / f"{rec.name}.csv")
        plot_steer = rec.name.startswith("1.4")
        save_plot(rec, run_dir / "plots" / f"{rec.name}.png", plot_steer=plot_steer, plot_yaw=False)
    write_summary(records, run_dir / "summary.txt")
    print(f"Done. Summary: {run_dir / 'summary.txt'}")


if __name__ == "__main__":
    main()
