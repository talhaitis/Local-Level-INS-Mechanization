#include "Utilities.h"
#include <fstream>
#include <sstream>
#include <iostream>

bool writeIMUDataCSV(const std::string& filename, const std::vector<IMURecord>& data) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing.\n";
        return false;
    }
    
    // Write CSV header
    outFile << "time,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n";
    
    // Write each record as a CSV line.
    for (const auto &record : data) {
        std::ostringstream line;
        line << record.time << ","
             << record.gyro_x << ","
             << record.gyro_y << ","
             << record.gyro_z << ","
             << record.accel_x << ","
             << record.accel_y << ","
             << record.accel_z << "\n";
        outFile << line.str();
    }
    
    outFile.close();
    return true;
}
