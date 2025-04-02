#ifndef INS_MECHANIZATION_H
#define INS_MECHANIZATION_H

#include <string>
#include <vector>
#include <Eigen/Dense>

struct IMURecord
{
    double time;    // in seconds
    double gyro_x;  // in rad/s
    double gyro_y;  // in rad/s
    double gyro_zl; // in rad/s
    double accel_x; // in m/s^2
    double accel_y; // in m/s^2
    double accel_z; // in m/^2
};

class INSMechanization
{
public:
    INSMechanization();

    // Set intial state: latitude (radians), longitude(radians), height(meters)
    // intitial velocity, and euler angle(roll, pitch, yaw) in radians
    void setInitialState(double latitude, double longitude, double height,
                         const Eigen::Vector3d &velocity,
                         const Eigen::Vector3d &eulerAngles);

    // Read the IMU data from a binary file
    bool readIMUData(const std::string &filename);

    // Runs the INS mechanization processing (attitude, velocity and position updates)
    void run();

    // Prints the results
    void printResulst() const;

private:
    // Container for IMU data
    std::vector<IMURecord> imuData;

    // Initial state variables
    double initLatitude;  // in radians
    double initLongitude; // in radians
    double initHeight;    // in meters

    Eigen::Vector3d initVelocity;
    Eigen::Vector3d initEulerAngles;

    // -------------------------------------------------------------------------------------------------------------
    // Constants (WGS84, gravity, etc.)
    // -------------------------------------------------------------------------------------------------------------

    // Earth parameters (WGS84)
    static constexpr double semiMajorAxis = 6378137.0; // [m]
    static constexpr double eccentricitySquared = 6.69438e-3;
    static constexpr double earthRotationRate = 7.292115147e-5; // [rad/s]

    // Gravity model coefficients
    static constexpr double a1 = 9.7803267715;
    static constexpr double a2 = 0.0052790414;
    static constexpr double a3 = 0.0000232718;
    static constexpr double a4 = -0.000003087691089;
    static constexpr double a5 = 0.000000004397731;
    static constexpr double a6 = 0.000000000000721;
};

#endif