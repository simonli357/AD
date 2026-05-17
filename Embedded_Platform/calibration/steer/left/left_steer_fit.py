import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import re


def format_cpp_float(value):
    return f"{float(value): .16f}f"


def update_steeringmotor_cpp(coefficients, side):
    if len(coefficients) != 6:
        raise ValueError("steeringmotor.cpp update expects order-5 coefficients")

    target = current_dir.parents[2] / "Embedded_2024" / "source" / "drivers" / "steeringmotor.cpp"
    text = target.read_text()

    condition = r"f_angle > 0" if side == "left" else r"f_angle < 0"
    pattern = re.compile(
        rf"(else if\({condition}\)\s*\{{\s*"
        rf"// Polynomial Coefficients \(Order 5\):\s*)"
        rf"const float a5 = .*?;\s*"
        rf"const float a4 = .*?;\s*"
        rf"const float a3 = .*?;\s*"
        rf"const float a2 = .*?;\s*"
        rf"const float a1 = .*?;\s*"
        rf"const float c  = .*?;",
        re.DOTALL,
    )

    a5, a4, a3, a2, a1, c = coefficients
    replacement = (
        r"\g<1>"
        f"const float a5 = {format_cpp_float(a5)};\n"
        f"            const float a4 = {format_cpp_float(a4)};\n"
        f"            const float a3 = {format_cpp_float(a3)};\n"
        f"            const float a2 = {format_cpp_float(a2)};\n"
        f"            const float a1 = {format_cpp_float(a1)};\n"
        f"            const float c  = {format_cpp_float(c)};"
    )

    updated, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise RuntimeError(f"Could not find the {side} steering coefficient block in {target}")

    target.write_text(updated)
    print(f"Updated {side} steering coefficients in {target}")


def prompt_update_steeringmotor(coefficients, side):
    response = input(f"\nUpdate {side} coefficients in steeringmotor.cpp? [y/N] ").strip().lower()
    if response in {"y", "yes"}:
        update_steeringmotor_cpp(coefficients, side)
    else:
        print("Skipped steeringmotor.cpp update.")


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
prompt_update_steeringmotor(coeff, "left")
plt.show()
