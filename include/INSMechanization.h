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
    double gyro_z;  // in rad/s
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
    void setInitialState(double latitude, double longitude, double height, double normalGravity,
                         const Eigen::Vector3d &velocity,
                         const Eigen::Vector3d &eulerAngles);

    // Read the IMU data from a binary file
    bool readIMUData(const std::string &filename);
    // Getter for the raw IMU data (modifiable reference)
    std::vector<IMURecord>& getIMUData();

    // Returns a pair: first is roll (rad), second is pitch (rad).
    Eigen::Vector3d computeInitialAlignment(double alignmentWindow = 120.0) const;

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
    double normalGravity;

    Eigen::Vector3d initVelocity;
    Eigen::Vector3d initEulerAngles;    


};

#endif