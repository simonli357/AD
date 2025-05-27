import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

order = 5

current_dir = Path(__file__).parent
df = pd.read_csv(current_dir / "left_steer.csv")

coeff = np.polyfit(df["Steer Angle"], df["Duty Cycle"], order)

print(f"\nPolynomial Coefficients (Order {order}):")
fit_eq = "Duty ="
for i, c in enumerate(coeff):
    power = order - i
    if power > 0:
        print(f"  a{power} = {c:.16f}")
        fit_eq += f" {c:+.7e}·x^{power}"
    else:
        print(f"  c     = {c:.16f}")
        fit_eq += f" {c:+.7e}"

buffer = 4  # degrees beyond the data range
angle_min = df["Steer Angle"].min() - buffer
angle_max = df["Steer Angle"].max() + buffer
angle_range = np.linspace(angle_min, angle_max, 300)

# Evaluate the polynomial fit
duty_fit = np.polyval(coeff, angle_range)

# Plotting
plt.figure(figsize=(8, 5))
plt.scatter(df["Steer Angle"], df["Duty Cycle"], label="Data", color="green")
plt.plot(angle_range, duty_fit, label=f"Order {order} Fit", color="orange")

# Mark original data range
plt.axvline(df["Steer Angle"].min(), color="gray", linestyle="--", linewidth=1, label="Data Range")
plt.axvline(df["Steer Angle"].max(), color="gray", linestyle="--", linewidth=1)

plt.xlabel("Steer Angle (Degrees)")
plt.ylabel("Duty Cycle")
plt.title(f"Duty Cycle vs Steer Angle (Order {order} Fit with Extrapolation)")
plt.grid(True)
plt.legend(loc="upper right")
plt.text(
    0.02, 0.02, fit_eq, transform=plt.gca().transAxes,
    fontsize=9, va='bottom', ha='left',
    bbox=dict(facecolor='white', edgecolor='gray', boxstyle='round,pad=0.4')
)
plt.tight_layout()
plt.savefig(current_dir / f"steer_angle_pwm_fit_order{order}.png", dpi=150)
plt.show()
