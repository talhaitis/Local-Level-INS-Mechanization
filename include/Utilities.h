#ifndef UTILITIES_H
#define UTILITIES_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include "INSMechanization.h"  


// Conversion functions (inline so they're header-only)
inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

inline double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

// Convert gyro bias from deg/hr to rad/s
inline double gyroBiasDegHrToRadSec(double degHr) {
    return deg2rad(degHr) / 3600.0;
}

// Convert accelerometer bias from micro-g (µg) to m/s² (assuming g ≈ 9.81 m/s²)
inline double accelBiasMicrogToMPerSec2(double microg) {
    return microg * 1e-6 * 9.81;
}

// CSV writing function declaration
bool writeIMUDataCSV(const std::string& filename, const std::vector<IMURecord>& data);

#endif // UTILITIES_H
