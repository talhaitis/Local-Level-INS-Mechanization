#include "INSMechanization.h"
#include <iostream>
#include <fstream>

INSMechanization::INSMechanization() : initLatitude(0.0), initLongitude(0.0), initHeight(0.0),
                                       initVelocity(Eigen::Vector3d::Zero()), initEulerAngles(Eigen::Vector3d::Zero())
{
    // Constructor initilization of the INS
}

void INSMechanization::setInitialState(double latitiude, double longitude, double height,
                                       const Eigen::Vector3d &velocity,
                                       const Eigen::Vector3d &eulerAngles)
{
    initLatitude = latitiude;
    initLongitude = longitude;
    initHeight = height;
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
std::vector<IMURecord>& INSMechanization::getIMUData() {
    return imuData;
}

void INSMechanization::run()
{
    std::cout << "Running INS Mechanization...." << std::endl;
    std::cout << "Initial Latitude (rad): " << initLatitude << std::endl;
    std::cout << "Initial Longitude (rad): " << initLongitude << std::endl;
    std::cout << "Initial Height(m): " << initHeight << std::endl;
    std::cout << "Number of IMU records: " << imuData.size() << std::endl;
}

void INSMechanization::printResulst() const
{
}