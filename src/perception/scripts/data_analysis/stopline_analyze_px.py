import os
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
import numpy as np

# Get the path to the CSV file in the same directory as this script
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, 'stopline2.csv')

# Read the CSV file (comma-separated)
df = pd.read_csv(csv_path, sep=',')
df.columns = df.columns.str.strip()  # Clean column names just in case

# Drop any rows with missing data
df.dropna(inplace=True)

# Display basic info
print("\n--- Data Preview ---")
print(df.head())

print("\n--- Summary Statistics ---")
print(df.describe())

# Correlation analysis
print("\n--- Correlation ---")
print(df.corr())

# Calculate error
df['error'] = df['measured'] - df['pixel']  # You can also reverse this if you prefer: df['error'] = df['pixel'] - df['measured']

# Linear regression using numpy polyfit (Now fitting measured as a function of pixel)
coeffs = np.polyfit(df['pixel'], df['measured'], deg=1)
polyfit_slope = coeffs[0]
polyfit_intercept = coeffs[1]

print("\n--- Linear Fit using numpy.polyfit ---")
print(f"Slope (a): {polyfit_slope:.4f}")
print(f"Intercept (b): {polyfit_intercept:.4f}")

# First Plot: Measured vs Pixel and Ideal Line
plt.figure()
plt.scatter(df['pixel'], df['measured'], label='Measured vs Pixel', marker='o')
plt.plot(df['pixel'], polyfit_slope * df['pixel'] + polyfit_intercept, label=f'Polyfit Line: y={polyfit_slope:.2f}x+{polyfit_intercept:.2f}', linestyle='--')
plt.xlabel('Pixel')
plt.ylabel('Measured')
plt.title('Measured vs Pixel')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# Second Plot: Error vs Pixel
plt.figure()
plt.plot(df['pixel'], df['error'], 'o-', label='Error (Measured - Pixel)')
plt.axhline(0, color='gray', linestyle='--')
plt.xlabel('Pixel')
plt.ylabel('Error')
plt.title('Error vs Pixel')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
