#!/usr/bin/env python3
"""Car function test.

Walks through every actuator/sensor on the embedded platform one at a time and
records telemetry. Results, CSVs and plots are written to a per-run timestamped
folder under ./car_function_results/.

Set 1  car on a box (wheels free):
    1.1  Communication check
    1.2  Forward speed ramp 0 → 40 cm/s → 0
    1.3  Reverse speed ramp 0 → -40 cm/s → 0
    1.4  Steering sweep at 5 cm/s: 0 → max-left → max-right → 0

Set 2  car on the ground:
    2.1  Drive ~4 m forward at 35 cm/s straight
    2.2  Half circle at max-left steering
    2.3  Half circle at max-right steering

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
DEFAULT_MAX_STEER_DEG = 20.0  # firmware-side limit; user can override
RUNAWAY_FACTOR = 2.5          # abort if |enc_speed| > factor * |cmd_speed| + 10 cm/s


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
    runaway_check: bool = True,
    label: str = "",
) -> bool:
    """Drive a parameterised profile for duration_s. Returns True on clean finish."""
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

        if runaway_check and abs(cmd_v) > 1.0 and not (link.last_enc_speed != link.last_enc_speed):
            limit = RUNAWAY_FACTOR * abs(cmd_v) + 10.0
            if abs(link.last_enc_speed) > limit:
                print(f"\n!! Runaway detected: enc={link.last_enc_speed:.1f} cm/s vs cmd={cmd_v:.1f} cm/s — aborting segment")
                link.stop()
                return False

        if label and int(elapsed * 2) != int((elapsed - 0.02) * 2):
            sys.stdout.write(f"\r  {label}: {elapsed:5.2f}/{duration_s:.2f}s  cmd={cmd_v:+6.2f} cm/s  steer={cmd_s:+6.2f}°  enc={link.last_enc_speed:+7.2f}")
            sys.stdout.flush()

        time.sleep(0.005)

    if label:
        sys.stdout.write("\n")
    return True


def hold(link: CarLink, record: TestRecord, speed: float, steer: float, seconds: float, label: str = "") -> bool:
    return run_segment(link, record, seconds, lambda t: speed, lambda t: steer, label=label)


def ramp(link: CarLink, record: TestRecord, v0: float, v1: float, steer: float, seconds: float, label: str = "") -> bool:
    return run_segment(
        link,
        record,
        seconds,
        lambda t, v0=v0, v1=v1, T=seconds: v0 + (v1 - v0) * (t / T),
        lambda t: steer,
        label=label,
    )


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
        # break wrap-around
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
        f.write(f"Car function test — {datetime.now().isoformat(timespec='seconds')}\n")
        f.write("=" * 60 + "\n")
        for r in records:
            status = "PASS" if r.user_pass else ("FAIL" if r.user_pass is False else "SKIP")
            f.write(f"[{status}] {r.name}\n")
            if r.user_note:
                f.write(f"       note: {r.user_note}\n")


# ────────────────────────────────────────────────────────────────────────────────
def test_communication(link: CarLink) -> TestRecord:
    rec = TestRecord(name="1.1_communication")
    print("\n=== Test 1.1 — Communication check ===")
    print("Listening for encoder/IMU telemetry for 3 s while sending zero command…")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        link.send(0.0, 0.0)
        link.drain()
        time.sleep(CMD_PERIOD_S)
    print(f"  Encoder messages seen : {'YES' if link.enc_seen else 'NO'}")
    print(f"  IMU messages seen     : {'YES' if link.imu_seen else 'NO'}")
    if not link.enc_seen:
        print("  WARNING: no encoder telemetry — downstream tests will be uninformative.")
    rec.user_pass = link.enc_seen and link.imu_seen
    if not rec.user_pass:
        missing = []
        if not link.enc_seen:
            missing.append("encoder")
        if not link.imu_seen:
            missing.append("imu")
        rec.user_note = "missing telemetry: " + ", ".join(missing)
    return rec


def test_forward_ramp(link: CarLink, vmax: float) -> TestRecord:
    rec = TestRecord(name="1.2_forward_ramp")
    print(f"\n=== Test 1.2 — Forward ramp 0 → {vmax:.0f} cm/s → 0 (car on box) ===")
    if not yes_no("Is the car on the box with wheels free?", True):
        rec.user_note = "user aborted: car not on box"
        rec.user_pass = False
        return rec
    ramp_t = 6.0
    if not ramp(link, rec, 0.0, vmax, 0.0, ramp_t, label="ramp-up"):
        link.stop()
        rec.user_pass = False
        rec.user_note = "runaway detected"
        return rec
    hold(link, rec, vmax, 0.0, 2.0, label="hold")
    ramp(link, rec, vmax, 0.0, 0.0, ramp_t, label="ramp-down")
    link.stop()
    rec.user_pass = True
    return rec


def test_reverse_ramp(link: CarLink, vmax: float) -> TestRecord:
    rec = TestRecord(name="1.3_reverse_ramp")
    print(f"\n=== Test 1.3 — Reverse ramp 0 → -{vmax:.0f} cm/s → 0 (car on box) ===")
    ramp_t = 6.0
    if not ramp(link, rec, 0.0, -vmax, 0.0, ramp_t, label="ramp-up"):
        link.stop()
        rec.user_pass = False
        rec.user_note = "runaway detected"
        return rec
    hold(link, rec, -vmax, 0.0, 2.0, label="hold")
    ramp(link, rec, -vmax, 0.0, 0.0, ramp_t, label="ramp-down")
    link.stop()
    rec.user_pass = True
    return rec


def test_steering_sweep(link: CarLink, max_steer: float) -> TestRecord:
    rec = TestRecord(name="1.4_steering_sweep")
    print(f"\n=== Test 1.4 — Steering sweep at 5 cm/s, ±{max_steer:.1f}° (car on box) ===")
    sweep_t = 4.0
    speed = 5.0
    # 0 → -max (left)
    steer_ramp(link, rec, speed, 0.0, -max_steer, sweep_t, label="→ left")
    hold(link, rec, speed, -max_steer, 1.5, label="hold left")
    # -max → +max
    steer_ramp(link, rec, speed, -max_steer, +max_steer, 2 * sweep_t, label="left → right")
    hold(link, rec, speed, +max_steer, 1.5, label="hold right")
    # +max → 0
    steer_ramp(link, rec, speed, +max_steer, 0.0, sweep_t, label="→ centre")
    link.stop()
    rec.user_pass = True
    return rec


def test_straight_4m(link: CarLink) -> TestRecord:
    rec = TestRecord(name="2.1_straight_4m")
    speed = 35.0
    distance = 400.0  # cm
    drive_t = distance / speed
    print(f"\n=== Test 2.1 — 4 m straight at {speed:.0f} cm/s ===")
    print("Place the car on the ground at the start mark, wheels straight.")
    if not yes_no("Is the car on the ground with a clear ~5 m runway?", True):
        rec.user_pass = False; rec.user_note = "user aborted: not on ground"; return rec
    ramp(link, rec, 0.0, speed, 0.0, 1.5, label="accel")
    hold(link, rec, speed, 0.0, drive_t, label="cruise")
    ramp(link, rec, speed, 0.0, 0.0, 1.0, label="decel")
    link.stop()
    measured = input("  → Measured forward distance [cm] (Enter to skip): ").strip()
    rec.user_note = f"measured_distance_cm={measured}" if measured else ""
    rec.user_pass = True
    return rec


def test_half_circle(link: CarLink, side: str, max_steer: float) -> TestRecord:
    sign = -1.0 if side == "left" else +1.0
    rec = TestRecord(name=f"2.{2 if side == 'left' else 3}_half_circle_{side}")
    speed = 25.0
    print(f"\n=== Test 2.{2 if side == 'left' else 3} — Half circle, max-{side} steer ===")
    print("First the steering will be set to max angle so you can place the car correctly.")
    if not yes_no("Pick up the car so the wheels are off the ground. Ready?", True):
        rec.user_pass = False; rec.user_note = "user aborted"; return rec
    # Pre-set the steering with wheels in the air (no drive)
    print(f"  Setting steering to {sign * max_steer:+.1f}° (no drive)…")
    hold(link, rec, 0.0, sign * max_steer, 2.0, label="set steer")
    print("  Steering set. Place the car on the ground at the start mark.")
    wait_enter("Place the car and press Enter when ready to drive")

    # Estimate half-circle drive time: assume circumference ~= pi * D, with D unknown.
    # Use a generous default of ~6 s at 25 cm/s; user interrupts with Ctrl+C if needed.
    drive_t = 8.0
    print(f"  Driving for ~{drive_t:.1f}s — press Ctrl+C to stop early.")
    try:
        ramp(link, rec, 0.0, speed, sign * max_steer, 1.0, label="accel")
        hold(link, rec, speed, sign * max_steer, drive_t, label="arc")
    except KeyboardInterrupt:
        print("\n  Interrupted by user.")
    ramp(link, rec, speed, 0.0, sign * max_steer, 1.0, label="decel")
    link.stop()
    diam = input(f"  → Measured arc diameter [cm] for {side} turn (Enter to skip): ").strip()
    rec.user_note = f"arc_diameter_cm={diam}" if diam else ""
    rec.user_pass = True
    return rec


# ────────────────────────────────────────────────────────────────────────────────
def main() -> None:
    ap = argparse.ArgumentParser(description="Interactive car function test")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--vmax", type=float, default=40.0, help="Max speed for set-1 ramps [cm/s]")
    ap.add_argument("--max-steer", type=float, default=DEFAULT_MAX_STEER_DEG, help="Max steering angle [deg]")
    ap.add_argument("--out-root", default=None, help="Root folder for results (default: ./car_function_results)")
    ap.add_argument("--skip-set1", action="store_true")
    ap.add_argument("--skip-set2", action="store_true")
    args = ap.parse_args()

    script_dir = Path(__file__).parent
    out_root = Path(args.out_root) if args.out_root else script_dir / "car_function_results"
    run_dir = out_root / datetime.now().strftime("run_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"Results will be saved to: {run_dir}")

    link = CarLink(args.port, args.baud)
    records: list[TestRecord] = []

    try:
        # ── Set 1 ──────────────────────────────────────────────────────────────
        if not args.skip_set1:
            print("\n############### SET 1 — car on box ###############")
            records.append(test_communication(link))
            records.append(test_forward_ramp(link, args.vmax))
            records.append(test_reverse_ramp(link, args.vmax))
            records.append(test_steering_sweep(link, args.max_steer))

        # ── Set 2 ──────────────────────────────────────────────────────────────
        if not args.skip_set2:
            print("\n############### SET 2 — car on ground ###############")
            records.append(test_straight_4m(link))
            records.append(test_half_circle(link, "left", args.max_steer))
            records.append(test_half_circle(link, "right", args.max_steer))

    except KeyboardInterrupt:
        print("\n!! Interrupted — stopping motor.")
    finally:
        link.close()

    # ── Save artefacts ─────────────────────────────────────────────────────────
    print("\nSaving CSVs and plots…")
    for rec in records:
        if not rec.samples:
            continue
        save_csv(rec, run_dir / "data" / f"{rec.name}.csv")
        plot_steer = rec.name.startswith("1.4") or rec.name.startswith("2.2") or rec.name.startswith("2.3")
        plot_yaw = rec.name.startswith("2.")
        save_plot(rec, run_dir / "plots" / f"{rec.name}.png", plot_steer=plot_steer, plot_yaw=plot_yaw)
    write_summary(records, run_dir / "summary.txt")
    print(f"Done. Summary: {run_dir / 'summary.txt'}")


if __name__ == "__main__":
    main()
