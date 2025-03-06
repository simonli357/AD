#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import pearsonr
import os

def analyze_sign_pose(csv_file):
    """
    Analyzes sign pose estimation errors by computing:
      - Lateral and longitudinal errors
      - Basic statistics (mean, std, min, max)
      - Correlation
      - RMSE
      - Scatter plots to visualize error behavior
    """

    # 1. Load data from CSV
    df = pd.read_csv(csv_file)

    # Check that required columns exist
    required_cols = ["measured_lateral", "measured_longitudinal",
                     "estimated_lateral", "estimated_longitudinal"]
    for col in required_cols:
        if col not in df.columns:
            raise ValueError(f"Required column '{col}' not found in CSV.")

    # 2. Compute error columns
    df["delta_lateral"] = df["estimated_lateral"] - df["measured_lateral"]
    df["delta_longitudinal"] = df["estimated_longitudinal"] - df["measured_longitudinal"]

    # 3. Basic statistics
    def print_stats(error_name, error_data):
        print(f"\n=== {error_name} ===")
        print(f"Mean: {np.mean(error_data):.4f}")
        print(f"Std Dev: {np.std(error_data):.4f}")
        print(f"Min: {np.min(error_data):.4f}")
        print(f"Max: {np.max(error_data):.4f}")

    print_stats("Lateral Error", df["delta_lateral"])
    print_stats("Longitudinal Error", df["delta_longitudinal"])

    # 4. Correlation analysis
    #    For instance, how lateral error correlates with measured longitudinal distance
    corr_lateral, p_lateral = pearsonr(df["measured_longitudinal"], df["delta_lateral"])
    corr_long, p_long = pearsonr(df["measured_longitudinal"], df["delta_longitudinal"])

    print("\n=== Correlation Analysis ===")
    print(f"Correlation (Measured Longitudinal vs. Lateral Error): r = {corr_lateral:.4f}, p = {p_lateral:.4e}")
    print(f"Correlation (Measured Longitudinal vs. Longitudinal Error): r = {corr_long:.4f}, p = {p_long:.4e}")

    # 5. RMSE
    rmse_lateral = np.sqrt(np.mean(df["delta_lateral"]**2))
    rmse_longitudinal = np.sqrt(np.mean(df["delta_longitudinal"]**2))
    print("\n=== RMSE ===")
    print(f"RMSE Lateral: {rmse_lateral:.4f}")
    print(f"RMSE Longitudinal: {rmse_longitudinal:.4f}")

    # 6. Create scatter plots
    fig, axs = plt.subplots(1, 3, figsize=(15, 5))

    # (a) Lateral Error vs. Measured Longitudinal
    axs[0].scatter(df["measured_longitudinal"], df["delta_lateral"], alpha=0.7)
    axs[0].set_title("Lateral Error vs. Measured Longitudinal")
    axs[0].set_xlabel("Measured Longitudinal (m)")
    axs[0].set_ylabel("Delta Lateral (m)")
    axs[0].grid(True)

    # (b) Longitudinal Error vs. Measured Longitudinal
    axs[1].scatter(df["measured_longitudinal"], df["delta_longitudinal"], alpha=0.7, color="orange")
    axs[1].set_title("Longitudinal Error vs. Measured Longitudinal")
    axs[1].set_xlabel("Measured Longitudinal (m)")
    axs[1].set_ylabel("Delta Longitudinal (m)")
    axs[1].grid(True)

    # (c) 2D error plot (delta_longitudinal vs. delta_lateral)
    axs[2].scatter(df["delta_lateral"], df["delta_longitudinal"], alpha=0.7, color="green")
    axs[2].set_title("2D Error Plot")
    axs[2].set_xlabel("Delta Lateral (m)")
    axs[2].set_ylabel("Delta Longitudinal (m)")
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

    # 7. Bin or segment data by measured longitudinal distance (optional)
    #    Example: bins of 5 m up to the max distance
    max_distance = df["measured_longitudinal"].max()
    bin_size = 5.0
    bins = np.arange(0, max_distance + bin_size, bin_size)
    df["distance_bin"] = pd.cut(df["measured_longitudinal"], bins, include_lowest=True)

    # Compute average lateral error in each bin
    binned_stats = df.groupby("distance_bin").agg(
        mean_lat_error=("delta_lateral", "mean"),
        std_lat_error=("delta_lateral", "std"),
        mean_long_error=("delta_longitudinal", "mean"),
        std_long_error=("delta_longitudinal", "std"),
        count=("delta_lateral", "count")
    ).reset_index()

    print("\n=== Binned Error Stats (by Measured Longitudinal) ===")
    print(binned_stats)

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file_path = os.path.join(current_dir, "output.csv")
    analyze_sign_pose(csv_file_path)
