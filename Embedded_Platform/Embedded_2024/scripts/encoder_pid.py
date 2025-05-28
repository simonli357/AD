#!/usr/bin/env python3
"""
Encoder test — PID / speed / PWM

* PID mode
  ─────────
  • Activates the steering‑PID once at start:
        #12:<active>:<Kp>:<Ki>:<Kd>;;
  • Then streams speed & angle with motor‑ID 13.
* Speed mode (default) → motor‑ID 11 (cm/s)
* Safe‑window PWM      → motor‑ID 10 (0.065–0.11)

All logging & post‑processing identical to the original script.
"""
from __future__ import annotations
import argparse, sys, time, re, csv
from pathlib import Path
import serial

# ────────────────────────────────────────────────────────────────────────────────
# Encoder patterns
ENC_PATTERN = re.compile(r"@5:([-0-9.]+);;")
TOT_PATTERN = re.compile(r"\[Encoder\]\s+Total displacement\s*=\s*([-0-9.]+)°")

MOTOR_ID_PWM   = 10     # duty‑cycle 0‑1
MOTOR_ID_SPEED = 11     # cm/s
MOTOR_ID_PID   = 12     # activate PID
MOTOR_ID_SET   = 13     # speed & angle once PID is active

PWM_MIN, PWM_MAX = 0.065, 0.11
BAND_TOL, HOLD_S, RAMP_SKIP_START = 0.15, 0.5, 0.0

# ───────────────────────── helpers ─────────────────────────────────────────────
def build_cmd(motor_id: int, *values: float) -> bytes:
    """Return encoded motor‑controller command."""
    payload = ":".join(f"{v:.4f}" for v in values)
    return f"#{motor_id}:{payload};;\r\n".encode()

# ───────────────────────── main test loop ─────────────────────────────────────
def run_test(port: str, baud: int,
             cmd_tuple: tuple[int, tuple[float, ...]],
             duration: float) -> tuple[list[float], list[float], float | None, float | None]:
    """
    *cmd_tuple* = (stream_id, stream_vals)
        • PID mode   → (MOTOR_ID_SET, (speed_cm_s, steer_deg))
        • Speed/PWM  → (motor_id,      (value,        steer_deg))
    """
    stream_id, stream_vals = cmd_tuple
    ser = serial.Serial(port, baudrate=baud, timeout=0.1)
    time.sleep(0.1)

    t0, next_send = time.time(), 0.0
    times, speeds = [], []
    total_start = total_end = None
    real_dur = duration + 1.0

    print(f"Running {real_dur:.2f}s … (motor {stream_id} values {stream_vals})")
    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if elapsed >= real_dur:
                break
                
            if elapsed >= next_send:
                ser.write(build_cmd(stream_id, *stream_vals))
                next_send += 0.1                       # 10 Hz update

            while ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()

                if m := ENC_PATTERN.match(line):
                    speeds.append(float(m.group(1)))
                    times.append(elapsed)
                elif m := TOT_PATTERN.match(line):
                    # deg = float(m.group(1))
                    # if elapsed >= RAMP_SKIP_START and total_start is None:
                    #     total_start = deg
                    pass
                    total_end = deg

            if int(elapsed * 10) % 10 == 0:
                sys.stdout.write(f"\r{elapsed:6.2f}s / {real_dur:.2f}s")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n⚙️  Ctrl‑C — stopping motor immediately.")
        ser.write(build_cmd(stream_id, 0.0, 0.0))
        raise
    finally:
        try:
            ser.write(build_cmd(stream_id, 0.0, 0.0))
        except Exception:
            pass
        ser.close()
        print("\nSerial closed.")

    return times, speeds, total_start, total_end

# ───────────────────────── analysis helpers (unchanged) ───────────────────────
def estimate_delay(t, v, ref):
    import numpy as np
    if not t: return float("nan")
    t, v = map(np.asarray, (t, v))
    in_band = np.abs(v - ref) <= BAND_TOL * abs(ref)
    if not in_band.any(): return t[-1]
    idx = np.where(in_band)[0]
    for seg in np.split(idx, np.where(np.diff(idx) != 1)[0] + 1):
        if t[seg[-1]] - t[seg[0]] >= HOLD_S:
            return t[seg[0]]
    return t[-1]

def compute_stats(t, v, start):
    import numpy as np
    if not t: return {k: float("nan") for k in ("mean","std","min_dev","max_dev")}
    t, v = map(np.asarray, (t, v))
    seg = v[t >= start]
    if not seg.size: return {k: float("nan") for k in ("mean","std","min_dev","max_dev")}
    mean = float(seg.mean()); resid = seg - mean
    return {"mean": mean, "std": float(resid.std(ddof=1)),
            "min_dev": float(resid.min()), "max_dev": float(resid.max())}

def save_csv(t, v, path):
    with open(path, "w", newline="") as f:
        csv.writer(f).writerows(zip(("time_s","enc_speed_cm_s"),*zip(t,v)))
    print(f"CSV saved ➜ {path}")

def save_plot(t, v, ref, delay, stats, path):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot."); return
    plt.figure(figsize=(10,5))
    plt.plot(t, v, label="Measured (cm/s)")
    plt.axhline(ref,          linestyle=":",  label=f"Cmd {ref:.1f} cm/s")
    plt.axvline(delay,        linestyle="-.", label=f"Delay {delay:.2f}s")
    plt.axhline(stats["mean"],linestyle="--", label=f"Mean {stats['mean']:.2f}")
    plt.fill_between(t, stats["mean"]-stats["std"], stats["mean"]+stats["std"],
                     alpha=.25, label=f"±1σ {stats['std']:.2f}")
    plt.xlabel("Time [s]"); plt.ylabel("Speed [cm/s]")
    plt.title("Encoder response (cm/s)"); plt.grid(linestyle=":")
    plt.legend(); plt.tight_layout(); plt.savefig(path,dpi=150)
    print(f"Plot saved ➜ {path}")

# ────────────────────────────── CLI ───────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser("Encoder constant‑command test (PID‑capable)")
    ap.add_argument("--cmd",  type=float, default=30.0,  help="Speed [cm/s]")
    ap.add_argument("--steer",type=float, default= -10.0, help="Steering angle [deg]")
    ap.add_argument("--pwm",  type=float, default=-1,   help=f"Duty cycle (0‑1), safe range {PWM_MIN}-{PWM_MAX}")
    ap.add_argument("--dur",  type=float, default=35.0, help="Duration [s]")
    ap.add_argument("--kp",   type=float, default=1.0,  help="PID Kp")
    ap.add_argument("--ki",   type=float, default=0.0,  help="PID Ki")
    ap.add_argument("--kd",   type=float, default=0.0,  help="PID Kd")
    ap.add_argument("--pid_off", action="store_true",   help="Send pid_active = 0 (deactivate)")
    ap.add_argument("--csv",  type=Path, help="Save raw data to CSV")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int,  default=115200)
    args = ap.parse_args()

    # ───────── determine mode ────────────────────────────────────────────
    use_pid = args.pwm == -1          # if --pwm used → PWM / speed modes
    if args.pwm != -1 and not (PWM_MIN <= args.pwm <= PWM_MAX):
        sys.exit(f"❌ PWM {args.pwm} outside safe range {PWM_MIN}-{PWM_MAX}.")
    if use_pid:
        # ① activate / deactivate once
        ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.1)
        time.sleep(0.1)
        active = 0.0 if args.pid_off else 1.0
        ser.write(build_cmd(MOTOR_ID_PID, active, args.kp, args.ki, args.kd))
        ack = ser.readline().decode(errors="ignore").strip()
        print(f"PID activation sent → “{ack}”")
        ser.close()
        # ② stream speed & angle through ID 13
        cmd_tuple = (MOTOR_ID_SET, (args.cmd, args.steer))
    else:
        motor = MOTOR_ID_PWM if args.pwm != -1 else MOTOR_ID_SPEED
        cmd_tuple = (motor, (args.pwm if motor == MOTOR_ID_PWM else args.cmd,
                             args.steer))

    try:
        times, speeds, t0, t1 = run_test(args.port, args.baud,
                                         cmd_tuple, args.dur)
    except KeyboardInterrupt:
        print("Test aborted by user."); return

    # ───────── print / save results ───────────────────────────────────────
    print()
    if speeds and cmd_tuple[0] == MOTOR_ID_SPEED:   # delay stats only for open‑loop speed
        delay = estimate_delay(times, speeds, args.cmd)
        stats = compute_stats(times, speeds, delay)
        print(f"Delay to ±{BAND_TOL*100:.0f}% band : {delay:.3f} s")
        print(f"Mean encoder speed        : {stats['mean']:.3f} cm/s")
        print(f"Noise σ                   : {stats['std']:.3f} cm/s")
        print(f"Min deviation             : {stats['min_dev']:.3f} cm/s")
        print(f"Max deviation             : {stats['max_dev']:.3f} cm/s")
    elif cmd_tuple[0] == MOTOR_ID_SET:
        print("PID mode — delay/noise stats skipped.")
    else:
        print("PWM mode — delay/noise stats skipped.")

    if t0 is not None and t1 is not None:
        disp = t1 - t0
        print(f"Net displacement          : {disp:.2f} °  "
              f"≈ {disp/-146:.2f} cm  "
              f"({disp/-146/args.dur:.2f} cm/s)"
              f"start: {t0:.2f}°  end: {t1:.2f}°")
    else:
        print("⚠️  No cumulative‑angle data received — displacement unavailable.")

    if args.csv and times:
        save_csv(times, speeds, args.csv)

    if cmd_tuple[0] == MOTOR_ID_SPEED and speeds:
        script_dir = Path(__file__).parent
        delay = estimate_delay(times, speeds, args.cmd)
        stats = compute_stats(times, speeds, delay)
        save_plot(times, speeds, args.cmd, delay,
                  stats, script_dir / "encoder_noise_plot.png")


if __name__ == "__main__":
    main()
