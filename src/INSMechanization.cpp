#include "INSMechanization.h"
#include <iostream>
#include <fstream>

INSMechanization::INSMechanization() : initLatitude(0.0), initLongitude(0.0), initHeight(0.0), normalGravity(9.8),
                                       initVelocity(Eigen::Vector3d::Zero()), initEulerAngles(Eigen::Vector3d::Zero())
{
    // Constructor initilization of the INS
}

void INSMechanization::setInitialState(double latitiude, double longitude, double height, double normalGravity,
                                       const Eigen::Vector3d &velocity,
                                       const Eigen::Vector3d &eulerAngles

)
{
    initLatitude = latitiude;
    initLongitude = longitude;
    initHeight = height;
    this->normalGravity = normalGravity;
    initVelocity = velocity;
    initEulerAngles = eulerAngles;
}

bool INSMechanization::readIMUData(const std::string &filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file)
    {
        std::cerr << "Error:Cannot open file" << filename << std::endl;
        return false;
    }
    IMURecord record;

    while (file.read(reinterpret_cast<char *>(&record), sizeof(IMURecord)))
    {
        imuData.push_back(record);
    }
    file.close();
    std::cout << "Read " << imuData.size() << " records from" << filename << " \n";
    return true;
}

// Getter for the raw IMU data
std::vector<IMURecord> &INSMechanization::getIMUData()
{
    return imuData;
}
// Compute initial alignment (roll, pitch, azimuth) based on data within alignmentWindow seconds
Eigen::Vector3d INSMechanization::computeInitialAlignment(double alignmentWindow) const
{
    if (imuData.empty()) {
        std::cerr << "No IMU data available for alignment." << std::endl;
        return Eigen::Vector3d::Zero();
    }

    double sum_fx = 0.0, sum_fy = 0.0, sum_fz = 0.0;
    double sum_wx = 0.0, sum_wy = 0.0, sum_wz = 0.0;
    int count = 0;
    for (const auto &record : imuData) {
        if (record.time <= alignmentWindow) {
            sum_fx += record.accel_x;
            sum_fy += record.accel_y;
            sum_fz += record.accel_z;
            
            sum_wx += record.gyro_x;
            sum_wy += record.gyro_y;
            sum_wz += record.gyro_z;
            ++count;
        }
    }
    if (count == 0) {
        std::cerr << "No data in the specified alignment window." << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    double avg_fx = sum_fx / count;
    double avg_fy = sum_fy / count;
    double avg_fz = sum_fz / count;

    double avg_wx = sum_wx / count;
    double avg_wy = sum_wy / count;
    // double avg_wz = sum_wz / count;

    // Accelerometer leveling for roll and pitch using computed gravity
    double sign_fz = (avg_fz >= 0.0) ? 1.0 : -1.0;
    double roll = -sign_fz * std::asin(avg_fy / normalGravity);
    double pitch = -sign_fz * std::asin(avg_fx / normalGravity);

    // Gyro compassing for azimuth: using atan2 of average gyro measurements (adjust sign/order as needed)
    double azimuth = std::atan2(avg_wx, avg_wy);
    
    // Return the three angles in a vector: [roll, pitch, azimuth] in radians.
    return Eigen::Vector3d(roll, pitch, azimuth);
}

void INSMechanization::run()
{
    std::cout << "Running INS Mechanization...." << std::endl;
    std::cout << "Initial Latitude (rad): " << initLatitude << std::endl;
    std::cout << "Initial Longitude (rad): " << initLongitude << std::endl;
    std::cout << "Initial Height (m): " << initHeight << std::endl;
    std::cout << "Number of IMU records: " << imuData.size() << std::endl;
    
    // Compute initial alignment using a time window (e.g., first 120 seconds)
    // The function returns a vector: [roll, pitch, azimuth] in radians.
    Eigen::Vector3d alignment = computeInitialAlignment(120.0);
    
    double roll_deg = alignment[0] * 180.0 / M_PI;
    double pitch_deg = alignment[1] * 180.0 / M_PI;
    double azimuth_deg = alignment[2] * 180.0 / M_PI;
    
    std::cout << "Initial Alignment:" << std::endl;
    std::cout << "  Roll: " << roll_deg << " degrees" << std::endl;
    std::cout << "  Pitch: " << pitch_deg << " degrees" << std::endl;
    std::cout << "  Azimuth: " << azimuth_deg << " degrees" << std::endl;
    
    // Optional: If you want to update alignment periodically, you could include a loop like:
    /*
    int totalTime = 600; // Total time in seconds
    for (int t = 0; t < totalTime; t++) {
        // Here, you could use a moving window (e.g., the past 120 seconds) to update alignment
        Eigen::Vector3d currentAlignment = computeInitialAlignment(120.0);
        double currentRoll = currentAlignment[0] * 180.0 / M_PI;
        double currentPitch = currentAlignment[1] * 180.0 / M_PI;
        double currentAzimuth = currentAlignment[2] * 180.0 / M_PI;
        std::cout << "Time " << t << " sec: "
                  << "Roll = " << currentRoll << " deg, "
                  << "Pitch = " << currentPitch << " deg, "
                  << "Azimuth = " << currentAzimuth << " deg" << std::endl;
        // Perform INS integration updates here...
        // Sleep for 1 second or simulate time step...
    }
    */
    
    // Continue with further INS processing if needed...
}


void INSMechanization::printResulst() const
{
}

