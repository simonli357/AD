import pandas as pd
import matplotlib.pyplot as plt
import os

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

# First scatter plot: Error in Lateral vs. Measured Longitudinal
plt.figure(figsize=(8, 6))
plt.scatter(measured_longitudinal, error_lateral, alpha=0.7, label='Error in Lateral')
plt.xlabel('Measured Longitudinal')
plt.ylabel('Error in Lateral')
plt.legend()
plt.grid()
plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='black')
plt.savefig(os.path.join(current_dir, name + '_error_lateral.png'))
plt.show()

# Second scatter plot: Error in Longitudinal vs. Measured Longitudinal
plt.figure(figsize=(8, 6))
plt.scatter(measured_longitudinal, error_longitudinal, alpha=0.7, label='Error in Longitudinal')
plt.xlabel('Measured Longitudinal')
plt.ylabel('Error in Longitudinal')
plt.legend()
plt.grid()
plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='black')
plt.savefig(os.path.join(current_dir, name + '_error_longitudinal.png'))
plt.show()

# Third scatter plot: Error in Lateral vs. Measured Lateral
plt.figure(figsize=(8, 6))
plt.scatter(measured_lateral, error_lateral, alpha=0.7, label='Error in Lateral')
plt.xlabel('Measured Lateral')
plt.ylabel('Error in Lateral')
plt.legend()
plt.grid()
plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='black')
plt.savefig(os.path.join(current_dir, name + '_error_lateral_vs_lateral.png'))
plt.show()

# Fourth scatter plot: Error in Longitudinal vs. Measured Lateral
plt.figure(figsize=(8, 6))
plt.scatter(measured_lateral, error_longitudinal, alpha=0.7, label='Error in Longitudinal')
plt.xlabel('Measured Lateral')
plt.ylabel('Error in Longitudinal')
plt.legend()
plt.grid()
plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='black')
plt.savefig(os.path.join(current_dir, name + '_error_longitudinal_vs_lateral.png'))
plt.show()
