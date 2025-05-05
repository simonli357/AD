#!/usr/bin/env python3
"""Analyze speed‑sweep CSV logs for delay and noise.

Usage
-----
$ python analyze_speed_data.py  \
      --cmd cmd_log.csv \
      --meas meas_log.csv \
      [--plot]

Outputs to stdout:
  • Estimated delay (lag) in seconds between command and encoder response.
  • Overall noise (std‑dev of residuals) once delay is compensated.
  • Per‑step statistics (mean error, std‑dev) for each commanded speed level.

If ``--plot`` is given, a PNG *analysis_plot.png* is saved showing the
commanded profile, shifted measured response, and residuals.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Tuple, Dict, List

import numpy as np
import pandas as pd
from scipy.signal import correlate
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from pathlib import Path

# ────────────────────────────────────────────────────────────────────────────────
CURRENT_DIR = Path(__file__).parent.resolve()
def read_csv_pair(cmd_path: Path, meas_path: Path) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """Read the two CSVs and return dataframes."""
    cmd = pd.read_csv(cmd_path)
    meas = pd.read_csv(meas_path)
    if list(cmd.columns) != ["time_s", "cmd_cm_s"]:
        sys.exit(f"Unexpected columns in {cmd_path}; expected time_s, cmd_cm_s")
    if list(meas.columns) != ["time_s", "meas_cm_s"]:
        sys.exit(f"Unexpected columns in {meas_path}; expected time_s, meas_cm_s")
    return cmd, meas

# ────────────────────────────────────────────────────────────────────────────────

def estimate_delay(cmd: pd.DataFrame, meas: pd.DataFrame, max_lag_s: float = 2.0) -> float:
    """Estimate lag (s) via cross‑correlation, limited to ±max_lag_s."""
    # Resample both signals to a common uniform grid (e.g. 100 Hz) for XC.
    fs = 100  # Hz
    t_min = max(cmd.time_s.min(), meas.time_s.min())
    t_max = min(cmd.time_s.max(), meas.time_s.max())
    t_uniform = np.arange(t_min, t_max, 1 / fs)

    # Interpolate
    cmd_interp = np.interp(t_uniform, cmd.time_s, cmd.cmd_cm_s)
    meas_interp = np.interp(t_uniform, meas.time_s, meas.meas_cm_s)

    # Remove mean to focus on dynamic signal
    cmd_detr = cmd_interp - cmd_interp.mean()
    meas_detr = meas_interp - meas_interp.mean()

    corr = correlate(meas_detr, cmd_detr, mode="full")
    lags = np.arange(-len(cmd_detr) + 1, len(cmd_detr)) / fs

    # Limit search window
    mask = np.abs(lags) <= max_lag_s
    lags_win = lags[mask]
    corr_win = corr[mask]

    best_idx = np.argmax(corr_win)
    delay = lags_win[best_idx]
    return delay

# ────────────────────────────────────────────────────────────────────────────────

def align_measured(meas: pd.DataFrame, delay: float) -> pd.DataFrame:
    """Shift measured signal forward so it aligns with command timing."""
    meas_shifted = meas.copy()
    meas_shifted["time_s"] = meas_shifted.time_s + delay  # shift forward if delay>0
    return meas_shifted

# ────────────────────────────────────────────────────────────────────────────────

def compute_noise(cmd: pd.DataFrame, meas_aligned: pd.DataFrame) -> Dict[str, float]:
    """Return overall noise (std of residuals)."""
    # Interpolate measured to command timestamps for residual computation
    f = interp1d(meas_aligned.time_s, meas_aligned.meas_cm_s, bounds_error=False, fill_value="extrapolate")
    meas_at_cmd = f(cmd.time_s)
    residual = meas_at_cmd - cmd.cmd_cm_s
    noise_std = np.std(residual)
    return {"noise_std_cm_s": float(noise_std)}

# ────────────────────────────────────────────────────────────────────────────────

def per_step_stats(cmd: pd.DataFrame, meas_aligned: pd.DataFrame) -> pd.DataFrame:
    """Compute mean error & noise for each distinct commanded speed level."""
    # Identify change points in command profile
    cmd_diff = cmd.cmd_cm_s.diff().fillna(0)
    step_idx = cmd_diff.ne(0).cumsum()
    cmd["step_id"] = step_idx

    # Merge measured @ command times (aligned)
    f = interp1d(meas_aligned.time_s, meas_aligned.meas_cm_s, bounds_error=False, fill_value="extrapolate")
    cmd["meas_cm_s"] = f(cmd.time_s)
    cmd["residual"] = cmd.meas_cm_s - cmd.cmd_cm_s

    stats = cmd.groupby("step_id").agg(
        cmd_speed=("cmd_cm_s", "first"),
        mean_meas=("meas_cm_s", "mean"),
        mean_err=("residual", "mean"),
        noise_std=("residual", "std"),
        samples=("residual", "size"),
    )
    return stats

# ────────────────────────────────────────────────────────────────────────────────

def plot_results(cmd: pd.DataFrame, meas_aligned: pd.DataFrame, residual: np.ndarray) -> None:
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(cmd.time_s, cmd.cmd_cm_s, label="Command", linewidth=1.3)
    plt.plot(meas_aligned.time_s, meas_aligned.meas_cm_s, label="Measured (shifted)", linewidth=1.0)
    plt.ylabel("Speed [cm/s]")
    plt.legend()
    plt.grid(True, which="both", linestyle=":")

    plt.subplot(2, 1, 2)
    plt.plot(cmd.time_s, residual, label="Residual (meas‑cmd)", linewidth=1.0)
    plt.xlabel("Time [s]")
    plt.ylabel("Error [cm/s]")
    plt.grid(True, which="both", linestyle=":")
    plt.legend()

    plt.tight_layout()
    filename = CURRENT_DIR / "analysis_plot.png"
    plt.savefig(filename, dpi=150)
    print("Saved analysis_plot.png")

# ────────────────────────────────────────────────────────────────────────────────
CURRENT_DIR = Path(__file__).parent.resolve()
def main() -> None:
    plot = True
    cmd = CURRENT_DIR / "cmd_log.csv"
    meas = CURRENT_DIR / "meas_log.csv"
    cmd_df, meas_df = read_csv_pair(cmd, meas)

    delay = estimate_delay(cmd_df, meas_df)
    meas_aligned = align_measured(meas_df, delay)

    noise_info = compute_noise(cmd_df, meas_aligned)

    stats = per_step_stats(cmd_df, meas_aligned)

    # Residual for plotting if needed
    f = interp1d(meas_aligned.time_s, meas_aligned.meas_cm_s, bounds_error=False, fill_value="extrapolate")
    residual_full = f(cmd_df.time_s) - cmd_df.cmd_cm_s

    print("\n===== Analysis Results =====")
    print(f"Estimated delay  : {delay:.3f} s (measured lags command)")
    print(f"Overall noise σ  : {noise_info['noise_std_cm_s']:.3f} cm/s (std dev of residuals)")
    print("\nPer‑step statistics (after delay compensation):")
    print(stats.to_string(index=False, float_format="{:.3f}".format))

    if plot:
        plot_results(cmd_df, meas_aligned, residual_full)


if __name__ == "__main__":
    main()
