#include "INSMechanization.h"
#include "Utilities.h"       // For conversion functions and CSV writing
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
Eigen::Vector3d INSMechanization::computeInitialAlignment(const IMUCalibration &calib, double alignmentWindow) const
{
    if (imuData.empty()) {
        std::cerr << "No IMU data available for alignment." << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    double sum_fx = 0.0, sum_fy = 0.0, sum_fz = 0.0;
    double sum_wx = 0.0, sum_wy = 0.0;
    int count = 0;
    
    for (const auto &record : imuData) {
        if (record.time <= alignmentWindow) {
            // Apply calibration corrections for each measurement:
            Eigen::Vector3d rawAccel(record.accel_x, record.accel_y, record.accel_z);
            Eigen::Vector3d rawGyro(record.gyro_x, record.gyro_y, record.gyro_z);
            Eigen::Vector3d correctedAccel = calib.correctAccelerometer(rawAccel);
            Eigen::Vector3d correctedGyro  = calib.correctGyro(rawGyro);
            
            // Accumulate the corrected values:
            sum_fx += correctedAccel.x();
            sum_fy += correctedAccel.y();
            sum_fz += correctedAccel.z();
            
            sum_wx += correctedGyro.x();
            sum_wy += correctedGyro.y();
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
    // We can ignore wz for the azimuth calculation as needed
    
    // Compute roll and pitch using the calibrated accelerometer average:
    double sign_fz = (avg_fz >= 0.0) ? 1.0 : -1.0;
    double roll = -sign_fz * std::asin(avg_fy / normalGravity);
    double pitch = -sign_fz * std::asin(avg_fx / normalGravity);
    
    // Compute azimuth from the calibrated gyro average:
    double azimuth = std::atan2(avg_wx, avg_wy);
    
    return Eigen::Vector3d(roll, pitch, azimuth);
}

void INSMechanization::run()
{
    std::cout << "Running INS Mechanization..." << std::endl;
    std::cout << "Initial Latitude (rad): " << initLatitude << std::endl;
    std::cout << "Initial Longitude (rad): " << initLongitude << std::endl;
    std::cout << "Initial Height (m): " << initHeight << std::endl;
    std::cout << "Number of IMU records: " << imuData.size() << std::endl;
    
    // --- Create and Configure a Calibration Object for the Alignment Window ---
    IMUCalibration calib;
    // Example: Set calibration parameters (bias, scale, non-orthogonality) as determined in your static calibration phase.
    double gyroBias_degHour = 0.1;
    double accelBias_microG = 3;
    double gyroBias_radPerSec = gyroBiasDegHrToRadSec(gyroBias_degHour);
    double accelBias_mPerSec2 = accelBiasMicrogToMPerSec2(accelBias_microG);
    
    Eigen::Vector3d accelBias(accelBias_mPerSec2, accelBias_mPerSec2, accelBias_mPerSec2);
    Eigen::Vector3d accelScale(1.0, 1.0, 1.0);
    Eigen::Vector3d accelNonOrthogonality(0.0, 0.0, 0.0);
    calib.setAccelBias(accelBias);
    calib.setAccelScale(accelScale);
    calib.setAccelNonOrthogonality(accelNonOrthogonality);
    
    Eigen::Vector3d gyroBias(gyroBias_radPerSec, gyroBias_radPerSec, gyroBias_radPerSec);
    Eigen::Vector3d gyroScale(1.0, 1.0, 1.0);
    Eigen::Vector3d gyroNonOrthogonality(0.0, 0.0, 0.0);
    calib.setGyroBias(gyroBias);
    calib.setGyroScale(gyroScale);
    calib.setGyroNonOrthogonality(gyroNonOrthogonality);
    
    // --- Compute the Initial Alignment using the calibrated values over the first 120 seconds ---
    Eigen::Vector3d alignment = computeInitialAlignment(calib, 120.0);
    double roll_deg  = alignment[0] * 180.0 / M_PI;
    double pitch_deg = alignment[1] * 180.0 / M_PI;
    double azimuth_deg = alignment[2] * 180.0 / M_PI;
    
    std::cout << "Initial Alignment (from first 120 sec of calibrated data):" << std::endl;
    std::cout << "  Roll: " << roll_deg << " deg" << std::endl;
    std::cout << "  Pitch: " << pitch_deg << " deg" << std::endl;
    std::cout << "  Azimuth: " << azimuth_deg << " deg" << std::endl;
    
    // Here you could add a loop to continuously update calibration and alignment per measurement.
    // For now, this run() function demonstrates the initial alignment process.
}


void INSMechanization::printResulst() const
{
}

