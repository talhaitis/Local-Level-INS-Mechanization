#include "IMUCalibration.h"

// Default constructor: initialize biases to zero and calibration parameters to zero.
IMUCalibration::IMUCalibration()
    : accelBias(Eigen::Vector3d::Zero()),
      accelScale(Eigen::Vector3d::Zero()),
      accelNonOrthogonality(Eigen::Vector3d::Zero()),
      gyroBias(Eigen::Vector3d::Zero()),
      gyroScale(Eigen::Vector3d::Zero()),
      gyroNonOrthogonality(Eigen::Vector3d::Zero())
{
}

// Accelerometer setters
void IMUCalibration::setAccelBias(const Eigen::Vector3d &bias)
{
    accelBias = bias;
}

void IMUCalibration::setAccelScale(const Eigen::Vector3d &scale)
{
    accelScale = scale;
}

void IMUCalibration::setAccelNonOrthogonality(const Eigen::Vector3d &nonOrthogonality)
{
    accelNonOrthogonality = nonOrthogonality;
}

// Gyroscope setters
void IMUCalibration::setGyroBias(const Eigen::Vector3d &bias)
{
    gyroBias = bias;
}

void IMUCalibration::setGyroScale(const Eigen::Vector3d &scale)
{
    gyroScale = scale;
}

void IMUCalibration::setGyroNonOrthogonality(const Eigen::Vector3d &nonOrthogonality)
{
    gyroNonOrthogonality = nonOrthogonality;
}

// Accelerometer getters
Eigen::Vector3d IMUCalibration::getAccelBias() const
{
    return accelBias;
}

Eigen::Vector3d IMUCalibration::getAccelScale() const
{
    return accelScale;
}

Eigen::Vector3d IMUCalibration::getAccelNonOrthogonality() const
{
    return accelNonOrthogonality;
}

// Gyroscope getters
Eigen::Vector3d IMUCalibration::getGyroBias() const
{
    return gyroBias;
}

Eigen::Vector3d IMUCalibration::getGyroScale() const
{
    return gyroScale;
}

Eigen::Vector3d IMUCalibration::getGyroNonOrthogonality() const
{
    return gyroNonOrthogonality;
}

// Correction function for the accelerometer:
// correctedAccel = (I + diag(accelScale + accelNonOrthogonality))^{-1} * (rawAccel - accelBias)
Eigen::Vector3d IMUCalibration::correctAccelerometer(const Eigen::Vector3d &rawAccel) const
{
    // Create a correction matrix using the diagonal of (scale + non-orthogonality)
    // Eigen::Matrix3d correctionMatrix = Eigen::Matrix3d::Identity();
    // correctionMatrix.diagonal() += (accelScale + accelNonOrthogonality);

    // return correctionMatrix.inverse() * (rawAccel - accelBias);
    return (rawAccel - accelBias);

}

// Correction function for the gyroscope:
// correctedGyro = (I + diag(gyroScale + gyroNonOrthogonality))^{-1} * (rawGyro - gyroBias)
Eigen::Vector3d IMUCalibration::correctGyro(const Eigen::Vector3d &rawGyro) const
{
    // Eigen::Matrix3d correctionMatrix = Eigen::Matrix3d::Identity();
    // correctionMatrix.diagonal() += (gyroScale + gyroNonOrthogonality);

    // return correctionMatrix.inverse() * (rawGyro - gyroBias);
    return (rawGyro - gyroBias);

}
