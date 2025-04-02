import os
import pandas as pd
import matplotlib.pyplot as plt

# Get the directory of the current script
script_dir = os.path.dirname(os.path.realpath(__file__))

# Build paths to the CSV files in the 'results' folder located at the project root
raw_csv_path = os.path.join(script_dir, "..", "results", "raw_data.csv")
calibrated_csv_path = os.path.join(script_dir, "..", "results", "calibrated_data.csv")

# Load the CSV files into Pandas DataFrames
raw_data = pd.read_csv(raw_csv_path)
calibrated_data = pd.read_csv(calibrated_csv_path)

# Create a figure with 2 rows and 3 columns of subplots
fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(18, 10), sharex=True)

# Define axis labels for clarity
axis_names = ['X', 'Y', 'Z']

# Plot Accelerometer data (row 0)
for i, axis in enumerate(axis_names):
    ax = axes[0, i]
    ax.plot(raw_data['time'], raw_data[f'accel_{axis.lower()}'], label=f'Raw Accel {axis}', marker='o', linestyle='-')
    ax.plot(calibrated_data['time'], calibrated_data[f'accel_{axis.lower()}'], label=f'Calibrated Accel {axis}', marker='x', linestyle='--')
    ax.set_title(f'Accelerometer {axis}-axis')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.legend()
    ax.grid(True)

# Plot Gyroscope data (row 1)
for i, axis in enumerate(axis_names):
    ax = axes[1, i]
    ax.plot(raw_data['time'], raw_data[f'gyro_{axis.lower()}'], label=f'Raw Gyro {axis}', marker='o', linestyle='-')
    ax.plot(calibrated_data['time'], calibrated_data[f'gyro_{axis.lower()}'], label=f'Calibrated Gyro {axis}', marker='x', linestyle='--')
    ax.set_title(f'Gyroscope {axis}-axis')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Rate (rad/s)')
    ax.legend()
    ax.grid(True)

# Adjust layout for better spacing
plt.tight_layout()

# Optionally, save the figure
output_path = os.path.join(script_dir, "..", "results", "imu_comparison_subplots.png")
plt.savefig(output_path)
print(f"Plot saved to {output_path}")

plt.show()
