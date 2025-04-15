#include <iostream>
#include "INSMechanization.h"
#include "Constants.h"       // For INIT_LAT_DEG, INIT_LONG_DEG, INIT_HEIGHT, DEG_TO_RAD, etc.
#include <Eigen/Dense>
#include <cmath>
#include <vector>

int main()
{
    INSMechanization ins;

    // ---------------------------------------------------------------------------
    // Set initial conditions from Constants.h
    // ---------------------------------------------------------------------------
    // Use constants defined in Constants.h (for example, INIT_LAT_DEG, INIT_LONG_DEG, INIT_HEIGHT)
    double initLat_deg = INIT_LAT_DEG;      // e.g., 51.07995352
    double initLong_deg = INIT_LONG_DEG;    // e.g., -114.13371127
    double initHeight = INIT_HEIGHT;        // e.g., 1118.502

    // Convert degrees to radians using constant conversion factor DEG_TO_RAD from Constants.h
    double initLat_rad = initLat_deg * DEG_TO_RAD;
    double initLong_rad = initLong_deg * DEG_TO_RAD;

    // Initial velocity and Euler angles are zero for a static test
    Eigen::Vector3d initVelocity(0.0, 0.0, 0.0);



    // Set the initial state in the INS object
    ins.setInitialState(initLat_rad, initLong_rad, initHeight, initVelocity);

    // ---------------------------------------------------------------------------
    // Read the raw IMU binary data (no calibration applied yet)
    // ---------------------------------------------------------------------------
    if (!ins.readIMUData("../data/project_data.BIN"))
    {
        std::cerr << "Failed to read IMU data." << std::endl;
        return 1;
    }

    // // Save a copy of the raw data for CSV output before calibration
    // std::vector<IMURecord> rawDataCopy = ins.getIMUData();
    // if (!writeIMUDataCSV("../results/raw_data.csv", rawDataCopy))
    // {
    //     std::cerr << "Failed to write raw data CSV." << std::endl;
    //     return 1;
    // }
    // std::cout << "Raw data written to raw_data.csv" << std::endl;

    // ---------------------------------------------------------------------------
    // Configure Calibration Parameters
    // // ---------------------------------------------------------------------------
    // IMUCalibration calibration;

    // // Given calibration values (need conversion):
    // double gyroBias_degHour = 0.1; // Gyro bias in deg/hr
    // double accelBias_microG = 3;   // Accelerometer bias in micro-g (µg)

    // // Convert gyro bias: deg/hr -> rad/s using conversion function from Utilities.h
    // double gyroBias_radPerSec = gyroBiasDegHrToRadSec(gyroBias_degHour);
    // // Convert accelerometer bias: µg -> m/s² (assuming g ≈ 9.81 m/s²)
    // double accelBias_mPerSec2 = accelBiasMicrogToMPerSec2(accelBias_microG);

    // std::cout << "Converted Accel Bias: " << accelBias_mPerSec2 << " m/s²" << std::endl;

    // // Set accelerometer calibration parameters:
    // Eigen::Vector3d accelBias(accelBias_mPerSec2, accelBias_mPerSec2, accelBias_mPerSec2);
    // Eigen::Vector3d accelScale(1.0, 1.0, 1.0);  // Nominal scale factors (no error)
    // Eigen::Vector3d accelNonOrthogonality(0.0, 0.0, 0.0); // Assume ideal alignment

    // calibration.setAccelBias(accelBias);
    // calibration.setAccelScale(accelScale);
    // calibration.setAccelNonOrthogonality(accelNonOrthogonality);

    // // Set gyroscope calibration parameters:
    // Eigen::Vector3d gyroBias(gyroBias_radPerSec, gyroBias_radPerSec, gyroBias_radPerSec);
    // Eigen::Vector3d gyroScale(1.0, 1.0, 1.0);  // Nominal scale factors
    // Eigen::Vector3d gyroNonOrthogonality(0.0, 0.0, 0.0); // Assume ideal alignment

    // calibration.setGyroBias(gyroBias);
    // calibration.setGyroScale(gyroScale);
    // calibration.setGyroNonOrthogonality(gyroNonOrthogonality);

    // // ---------------------------------------------------------------------------
    // // Calibrate the raw IMU data in main.cpp
    // // ---------------------------------------------------------------------------
    // std::vector<IMURecord>& rawData = ins.getIMUData();
    // for (size_t i = 0; i < rawData.size(); ++i)
    // {
    //     IMURecord& record = rawData[i];

    //     // Convert raw measurements to Eigen vectors
    //     Eigen::Vector3d rawAccel(record.accel_x, record.accel_y, record.accel_z);
    //     Eigen::Vector3d rawGyro(record.gyro_x, record.gyro_y, record.gyro_z);

    //     // Apply calibration corrections using the calibration object
    //     Eigen::Vector3d correctedAccel = calibration.correctAccelerometer(rawAccel);
    //     Eigen::Vector3d correctedGyro  = calibration.correctGyro(rawGyro);

    //     // Update the record with corrected values
    //     record.accel_x = correctedAccel.x();
    //     record.accel_y = correctedAccel.y();
    //     record.accel_z = correctedAccel.z();

    //     record.gyro_x = correctedGyro.x();
    //     record.gyro_y = correctedGyro.y();
    //     record.gyro_z = correctedGyro.z();
    // }

    // // Write the calibrated data to a CSV file
    // if (!writeIMUDataCSV("../results/calibrated_data.csv", ins.getIMUData()))
    // {
    //     std::cerr << "Failed to write calibrated data CSV." << std::endl;
    //     return 1;
    // }
    // std::cout << "Calibrated data written to calibrated_data.csv" << std::endl;

    // ---------------------------------------------------------------------------
    // Now, run the INS mechanization process using the calibrated data
    // ---------------------------------------------------------------------------
    ins.run();

    // ---------------------------------------------------------------------------
    // Print the INS results
    // ---------------------------------------------------------------------------
    ins.printResulst();

    return 0;
}
