# import os
# import pandas as pd
# import matplotlib.pyplot as plt

# # Get the directory of the current script
# script_dir = os.path.dirname(os.path.realpath(__file__))

# # Build paths to the CSV files in the 'results' folder located at the project root
# raw_csv_path = os.path.join(script_dir, "..", "results", "raw_data.csv")
# calibrated_csv_path = os.path.join(script_dir, "..", "results", "calibrated_data.csv")

# # Load the CSV files into Pandas DataFrames
# raw_data = pd.read_csv(raw_csv_path)
# calibrated_data = pd.read_csv(calibrated_csv_path)

# # Create a figure with 2 rows and 3 columns of subplots
# fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(18, 10), sharex=True)

# # Define axis labels for clarity
# axis_names = ['X', 'Y', 'Z']

# # Plot Accelerometer data (row 0)
# for i, axis in enumerate(axis_names):
#     ax = axes[0, i]
#     ax.plot(raw_data['time'], raw_data[f'accel_{axis.lower()}'], label=f'Raw Accel {axis}', marker='o', linestyle='-')
#     ax.plot(calibrated_data['time'], calibrated_data[f'accel_{axis.lower()}'], label=f'Calibrated Accel {axis}', marker='x', linestyle='--')
#     ax.set_title(f'Accelerometer {axis}-axis')
#     ax.set_xlabel('Time (s)')
#     ax.set_ylabel('Acceleration (m/s²)')
#     ax.legend()
#     ax.grid(True)

# # Plot Gyroscope data (row 1)
# for i, axis in enumerate(axis_names):
#     ax = axes[1, i]
#     ax.plot(raw_data['time'], raw_data[f'gyro_{axis.lower()}'], label=f'Raw Gyro {axis}', marker='o', linestyle='-')
#     ax.plot(calibrated_data['time'], calibrated_data[f'gyro_{axis.lower()}'], label=f'Calibrated Gyro {axis}', marker='x', linestyle='--')
#     ax.set_title(f'Gyroscope {axis}-axis')
#     ax.set_xlabel('Time (s)')
#     ax.set_ylabel('Angular Rate (rad/s)')
#     ax.legend()
#     ax.grid(True)

# # Adjust layout for better spacing
# plt.tight_layout()

# # Optionally, save the figure
# output_path = os.path.join(script_dir, "..", "results", "imu_comparison_subplots.png")
# plt.savefig(output_path)
# print(f"Plot saved to {output_path}")

# plt.show()

# import os
# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # Get the directory of the current script
# script_dir = os.path.dirname(os.path.realpath(__file__))

# # Load only the first 120 seconds of data
# csv_path = os.path.join(script_dir, "..", "results", "raw_data.csv")
# data = pd.read_csv(csv_path)

# # Filter data for the first two minutes (120 seconds)
# data = data[data['time'] <= 120]

# # Compute acceleration magnitude
# data['accel_mag'] = np.sqrt(data['accel_x']**2 + data['accel_y']**2 + data['accel_z']**2)

# # Calculate statistics
# mean_mag = data['accel_mag'].mean()
# std_mag = data['accel_mag'].std()

# print(f"[First 2 Minutes] Acceleration Magnitude Mean: {mean_mag:.4f} m/s²")
# print(f"[First 2 Minutes] Acceleration Magnitude Std Dev: {std_mag:.4f} m/s²")

# # Stationarity decision (adjust threshold if needed)
# threshold = 0.05
# if std_mag < threshold:
#     print("The IMU data appears to be stationary.")
# else:
#     print("The IMU data appears to be dynamic.")

# # Plot
# plt.figure(figsize=(10, 6))
# plt.plot(data['time'], data['accel_mag'], color='darkorange', label='Accel Magnitude')
# plt.xlabel('Time (s)')
# plt.ylabel('Acceleration Magnitude (m/s²)')
# plt.title('Acceleration Magnitude (First 2 Minutes)')
# plt.grid(True)
# plt.legend()


# plt.show()

import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

# --- Load Data ---
script_dir = os.path.dirname(os.path.realpath(__file__))
csv_path = os.path.join(script_dir, "..", "results", "position_output.csv")

df = pd.read_csv(csv_path)

# --- Plot Latitude vs Time ---
plt.figure(figsize=(10, 5))
plt.plot(df["time"], df["latitude_deg"], color='blue', marker='o', linestyle='-')
plt.xlabel("Time (s)")
plt.ylabel("Latitude (°)")
plt.title("Latitude Over Time")
plt.grid(True)
plt.gca().yaxis.set_major_formatter(ScalarFormatter(useMathText=False))  # Disable scientific notation
plt.tight_layout()
plt.savefig(os.path.join(script_dir, "..", "results", "latitude_over_time.png"))
plt.show()

# --- Plot Longitude vs Time ---
plt.figure(figsize=(10, 5))
plt.plot(df["time"], df["longitude_deg"], color='green', marker='x', linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("Longitude (°)")
plt.title("Longitude Over Time")
plt.grid(True)
plt.gca().yaxis.set_major_formatter(ScalarFormatter(useMathText=False))  # Disable scientific notation
plt.tight_layout()
plt.savefig(os.path.join(script_dir, "..", "results", "longitude_over_time.png"))
plt.show()

# --- Plot Height vs Time ---
plt.figure(figsize=(10, 5))
plt.plot(df["time"], df["height_m"], color='teal', marker='s', linestyle='-.')
plt.xlabel("Time (s)")
plt.ylabel("Height (m)")
plt.title("Altitude Over Time")
plt.grid(True)
plt.gca().yaxis.set_major_formatter(ScalarFormatter(useMathText=False))  # Disable scientific notation
plt.tight_layout()
plt.savefig(os.path.join(script_dir, "..", "results", "altitude_over_time.png"))
plt.show()
