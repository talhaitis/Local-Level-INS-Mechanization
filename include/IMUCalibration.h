#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <Eigen/Dense>

class IMUCalibration {
public:
    // Default constructor: initialize biases to zero and calibration matrices to zero.
    IMUCalibration();

    // Setters for accelerometer calibration parameters
    void setAccelBias(const Eigen::Vector3d& bias);
    void setAccelScale(const Eigen::Vector3d& scale);
    void setAccelNonOrthogonality(const Eigen::Vector3d& nonOrthogonality);

    // Setters for gyroscope calibration parameters
    void setGyroBias(const Eigen::Vector3d& bias);
    void setGyroScale(const Eigen::Vector3d& scale);
    void setGyroNonOrthogonality(const Eigen::Vector3d& nonOrthogonality);

    // Getters for accelerometer calibration parameters
    Eigen::Vector3d getAccelBias() const;
    Eigen::Vector3d getAccelScale() const;
    Eigen::Vector3d getAccelNonOrthogonality() const;

    // Getters for gyroscope calibration parameters
    Eigen::Vector3d getGyroBias() const;
    Eigen::Vector3d getGyroScale() const;
    Eigen::Vector3d getGyroNonOrthogonality() const;

    // Correction functions: 
    // corrected measurement = (I + calibration)^{-1} * (raw - bias)
    Eigen::Vector3d correctAccelerometer(const Eigen::Vector3d& rawAccel) const;
    Eigen::Vector3d correctGyro(const Eigen::Vector3d& rawGyro) const;

private:
    // Calibration parameters for accelerometer
    Eigen::Vector3d accelBias;           // b_a
    Eigen::Vector3d accelScale;          //S_a
    Eigen::Vector3d accelNonOrthogonality;   //N_a

    // Calibration parameters for gyroscope
    Eigen::Vector3d gyroBias;            // b_g
    Eigen::Vector3d gyroScale;          //S_g
    Eigen::Vector3d gyroNonOrthogonality;   //N_g

};

#endif // IMU_CALIBRATION_H
