#!/usr/bin/env python3
"""
Analyse longitudinal-error bias and scale in camera estimates.
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as st
import statsmodels.api as sm

# ----------------------------------------------------------------------
# 1. Load data
# ----------------------------------------------------------------------
current_dir = os.path.dirname(os.path.abspath(__file__))
name        = "0622"                        # CSV without extension
csv_path    = os.path.join(current_dir, f"{name}.csv")
df          = pd.read_csv(csv_path)

# Columns
mlat  = df["measured_lateral"].values
mlon  = df["measured_longitudinal"].values
elat  = df["estimated_lateral"].values
elon  = df["estimated_longitudinal"].values

# ----------------------------------------------------------------------
# 2. Residuals
# ----------------------------------------------------------------------
err_lat  = elat - mlat
err_lon  = elon - mlon          # <-- longitudinal residual we care about

# ----------------------------------------------------------------------
# 3. Constant-bias test  (one-sample t-test)
# ----------------------------------------------------------------------
mu_bias     = err_lon.mean()
sem         = err_lon.std(ddof=1) / np.sqrt(len(err_lon))
ci95_low, ci95_high = st.t.interval(0.95, len(err_lon)-1, loc=mu_bias, scale=sem)
t_stat, p_val = st.ttest_1samp(err_lon, 0.0)

print("\n=== CONSTANT-BIAS TEST (longitudinal) ===")
print(f"Mean bias      : {mu_bias:+.4f} m")
print(f"95 % CI        : [{ci95_low:+.4f} … {ci95_high:+.4f}] m")
print(f"t-stat / p-val : {t_stat:.2f}  /  {p_val:.3e}")
print("Reject H0 (no bias)?" , "YES" if p_val < 0.05 else "NO")

# ----------------------------------------------------------------------
# 4. Scale-and-bias fit  (OLS)
#     estimated_longitudinal = β0  +  β1 * measured_longitudinal
# ----------------------------------------------------------------------
X    = sm.add_constant(mlon)         # adds column of 1 → intercept β0
model = sm.OLS(elon, X).fit()
β0, β1 = model.params                # intercept, slope

print("\n=== OLS FIT ===")
print(f"β0 (intercept, constant bias) : {β0:+.4f} m")
print(f"β1 (slope, scale)             : {β1:.4f}")
print(model.summary().tables[1])     # small neat table

# ----------------------------------------------------------------------
# 5. Plots
# ----------------------------------------------------------------------
# 5-a  Longitudinal error vs measured range
plt.figure(figsize=(8,6))
plt.scatter(mlon, err_lon, alpha=0.7, label="Longitudinal error")
plt.axhline(mu_bias, color="tab:red", lw=1.5, label=f"mean bias {mu_bias:+.3f} m")

# regression line in *error* space:  (β0 + β1·mlon) – mlon  = β0 + (β1–1)·mlon
reg_err = β0 + (β1-1)*mlon
plt.plot(mlon, reg_err, "k--", label="regression (bias+scale)")

plt.xlabel("Measured longitudinal (m)")
plt.ylabel("Estimated - Measured (m)")
plt.title("Longitudinal residuals")
plt.legend()
plt.grid(True, which="both", ls=":")
plt.tight_layout()
png1 = os.path.join(current_dir, f"{name}_longitudinal_bias.png")
plt.savefig(png1, dpi=150)
plt.show()

# 5-b  (keep your original four error-scatter plots if you like)
# ----------------------------------------------------------------------

# Optional: Histogram of longitudinal errors
plt.figure(figsize=(6,4))
plt.hist(err_lon, bins=15, edgecolor="k", alpha=0.7)
plt.axvline(mu_bias, color="tab:red", lw=1.5, label=f"mean {mu_bias:+.3f}")
plt.xlabel("Longitudinal error (m)")
plt.title("Distribution of longitudinal errors")
plt.legend()
plt.tight_layout()
png2 = os.path.join(current_dir, f"{name}_longitudinal_hist.png")
plt.savefig(png2, dpi=150)
plt.show()

print(f"\nPlots saved to:\n  {png1}\n  {png2}")
