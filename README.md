# Local-Level INS Mechanization

## Overview
This repository contains a local-level Inertial Navigation System (INS) mechanization module implemented in C++. The goal is to process simulated IMU data (gyroscope and accelerometer readings) and produce estimates of position (latitude, longitude, height), velocity, and orientation (roll, pitch, yaw).

## Objectives
1. **Implement** local-level INS mechanization equations.
2. **Demonstrate** the results with position, velocity, and attitude estimates.
3. **Provide** well-documented C++ source code and thorough explanations.

## Data Description
-- **Data File**: `project_data.bin`
- **Format**:
  ```text
  time [sec], gyro_x [rad/s], gyro_y [rad/s], gyro_z [rad/s],
  accel_x [m/s^2], accel_y [m/s^2], accel_z [m/s^2]

- **Axes Orientation**:
- x-axis: pointing right (sideways)
- y-axis: pointing forward
- z-axis: pointing upward

## Key Parameters
- **IMU Data Rate**: 64 Hz

### Gyroscope
- Bias: 0.1 deg/hr  
- White noise (ARW): 0.01 deg/√hr  
- Bias instability (Gauss-Markov): 0.015 deg/hr  
- Correlation time: 1 hr

### Accelerometer
- Bias: 3 µg  
- White noise (VRW): 0.003 m/s/√hr  
- Bias instability (Gauss-Markov): 50 µg  
- Correlation time: 1 hr

### Initial Conditions
- Latitude: 51.07995352°  
- Longitude: -114.13371127°  
- Height: 1118.502 m  

### Earth Model (WGS84)
- Semi-major axis (a): 6378137 m  
- Eccentricity squared (e²): 6.69438e-3  
- Earth rotation rate (ωₑ): 7.292115147e-5 rad/s  
- Gravity model parameters for latitude- and height-dependent computation


## Building and Running
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/talhaitis/Local-Level-INS-Mechanization.git
   cd Local-Level-INS-Mechanization
