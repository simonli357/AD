import pandas as pd
import matplotlib.pyplot as plt
import os

# Load the CSV file
current_dir = os.path.dirname(os.path.abspath(__file__))
name = "old"
input_path = os.path.join(current_dir, name + '.csv')
df = pd.read_csv(input_path)

measured_lateral = df['measured_lateral']
measured_longitudinal = df['measured_longitudinal'] - 0.113
estimated_lateral = df['estimated_lateral']
estimated_longitudinal = df['estimated_longitudinal']
# First scatter plot: Measured vs. Estimated Lateral
plt.figure(figsize=(8, 6))
plt.scatter(measured_longitudinal, measured_lateral, alpha=0.7, label='Measured Lateral')
plt.scatter(measured_longitudinal, estimated_lateral, alpha=0.7, label='Estimated Lateral')
plt.xlabel('Measured Longitudinal')
plt.ylabel('Lateral Values')
plt.legend()
plt.grid()
plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='black')

# Save before showing
# plt.savefig('lateral_values.png')
plt.savefig(os.path.join(current_dir, name + '_lateral_values.png'))
plt.show()

# Second scatter plot: Measured vs. Estimated Longitudinal
plt.figure(figsize=(8, 6))
plt.scatter(measured_longitudinal, measured_longitudinal, alpha=0.7, label='Measured Longitudinal')
plt.scatter(measured_longitudinal, estimated_longitudinal, alpha=0.7, label='Estimated Longitudinal')
plt.xlabel('Measured Longitudinal')
plt.ylabel('Longitudinal Values')
plt.legend()
plt.grid()
plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='black')

# Save before showing
# plt.savefig('longitudinal_values.png')
plt.savefig(os.path.join(current_dir, name + '_longitudinal_values.png'))
plt.show()

# Third scatter plot: Measured vs. Estimated Lateral (Lateral vs. Lateral)
plt.figure(figsize=(8, 6))
plt.scatter(measured_lateral, measured_lateral, alpha=0.7, label='Measured Lateral')
plt.scatter(measured_lateral, estimated_lateral, alpha=0.7, label='Estimated Lateral')
plt.xlabel('Measured Lateral')
plt.ylabel('Lateral Values')
plt.legend()
plt.grid()
plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='black')

# Save before showing
# plt.savefig('lateral_vs_lateral.png')
plt.savefig(os.path.join(current_dir, name + '_lateral_vs_lateral.png'))
plt.show()

# Fourth scatter plot: Measured Longitudinal vs. Measured Lateral
plt.figure(figsize=(8, 6))
plt.scatter(measured_lateral, measured_longitudinal, alpha=0.7, label='Measured Longitudinal')
plt.scatter(estimated_lateral, estimated_longitudinal, alpha=0.7, label='Estimated Longitudinal')
plt.xlabel('Measured Lateral')
plt.ylabel('Measured Longitudinal')
plt.legend()
plt.grid()
plt.minorticks_on()
plt.grid(which='major', linestyle='-', linewidth='0.5', color='red')
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='black')

# Save before showing
# plt.savefig('longitudinal_vs_lateral.png')
plt.savefig(os.path.join(current_dir, name + '_longitudinal_vs_lateral.png'))
plt.show()
