import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# Load the CSV file
current_dir = os.path.dirname(os.path.abspath(__file__))
name = "test2"
input_path = os.path.join(current_dir, name + '.csv')
df = pd.read_csv(input_path)

measured_lateral = df['measured_lateral']
measured_longitudinal = df['measured_longitudinal']
estimated_lateral = df['estimated_lateral']
estimated_longitudinal = df['estimated_longitudinal']

# Calculate errors
error_lateral = measured_lateral - estimated_lateral
error_longitudinal = measured_longitudinal - estimated_longitudinal

# Function to plot scatter with linear fit
def plot_with_fit(x, y, xlabel, ylabel, filename, label):
    # Remove NaN and Inf values
    valid_mask = np.isfinite(x) & np.isfinite(y)
    x_valid = x[valid_mask]
    y_valid = y[valid_mask]
    
    if len(x_valid) > 1:  # Ensure there are enough points to fit a line
        plt.figure(figsize=(8, 6))
        plt.scatter(x_valid, y_valid, alpha=0.7, label=label)
        
        # Fit a linear line
        m, b = np.polyfit(x_valid, y_valid, 1)
        print(f"Linear Fit: y = {m:.2f}x + {b:.2f}")
        plt.plot(x_valid, m*x_valid + b, color='blue', linestyle='dashed', label='Linear Fit')
        
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid()
        plt.minorticks_on()
        plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
        plt.grid(which='minor', linestyle=':', linewidth='0.5', color='black')
        plt.savefig(os.path.join(current_dir, filename))
        plt.show()
    else:
        print(f"Skipping plot {filename}: Not enough valid data points.")

# # First scatter plot: Error in Lateral vs. Measured Longitudinal
# plot_with_fit(measured_longitudinal, error_lateral, 'Measured Longitudinal', 'Error in Lateral', name + '_error_lateral.png', 'Error in Lateral')

# Second scatter plot: Error in Longitudinal vs. Measured Longitudinal
plot_with_fit(estimated_longitudinal, error_longitudinal, 'Measured Longitudinal', 'Error in Longitudinal', name + '_error_longitudinal.png', 'Error in Longitudinal')

# # Third scatter plot: Error in Lateral vs. Measured Lateral
# plot_with_fit(measured_lateral, error_lateral, 'Measured Lateral', 'Error in Lateral', name + '_error_lateral_vs_lateral.png', 'Error in Lateral')

# # Fourth scatter plot: Error in Longitudinal vs. Measured Lateral
# plot_with_fit(measured_lateral, error_longitudinal, 'Measured Lateral', 'Error in Longitudinal', name + '_error_longitudinal_vs_lateral.png', 'Error in Longitudinal')
