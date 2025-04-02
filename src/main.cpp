// src/main.cpp
#include <iostream>
#include "INSMechanization.h"
#include <Eigen/Dense>
#include <cmath>
#include <filesystem>

int main()
{
    INSMechanization ins;
    // ---------------------------------------------------------------------------
    // Set initial conditions (provided in the project document)
    // ---------------------------------------------------------------------------

    double initLat_deg = 51.07995352;
    double initLong_deg = -114.13371127;
    double initHeight = 1118.502;

    // convert degress to radians
    const double deg2rad = M_PI / 180.0;
    double initLat_rad = initLat_deg * deg2rad;
    double initLong_rad = initLong_deg * deg2rad;

    // Initial velocity is zero for a static test
    Eigen::Vector3d initVelocity(0.0, 0.0, 0.0);
    Eigen::Vector3d initEulerAngles(0.0, 0.0, 0.0);

    // Set the initial state in the INS object
    ins.setInitialState(initLat_rad, initLong_rad, initHeight, initVelocity, initEulerAngles);

    // ---------------------------------------------------------------------------
    // Read the IMU binary data
    // ---------------------------------------------------------------------------

    if (!ins.readIMUData("../data/project_data.BIN"))
    {
        std::cerr << "Failed to read IMU data." << std::endl;
        return 1;
    }
    // ---------------------------------------------------------------------------
    // Run the INS mechanization process
    // ---------------------------------------------------------------------------
    ins.run();

    // ---------------------------------------------------------------------------
    // Print the results
    // ---------------------------------------------------------------------------
    ins.printResulst();

    return 0;
}
