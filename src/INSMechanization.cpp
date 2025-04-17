#include "INSMechanization.h"
#include "Utilities.h"
#include "Constants.h"
#include "EarthModel.h" // For computeNormalGravity, computeRadiusMeridian, computeRadiusPrimeVertical
#include <iostream>
#include <fstream>

INSMechanization::INSMechanization() : initLatitude(0.0), initLongitude(0.0), initHeight(0.0),
                                       initVelocity(Eigen::Vector3d::Zero())
{
    // Constructor initilization of the INS
}

void INSMechanization::setInitialState(double latitiude, double longitude, double height,
                                       const Eigen::Vector3d &velocity)
{
    initLatitude = latitiude;
    initLongitude = longitude;
    initHeight = height;
    initVelocity = velocity;
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
// In INSMechanization.cpp, replace the old time‐based version with:

Eigen::Vector3d INSMechanization::computeInitialAlignment(
    const IMUCalibration &calib,
    double normalGravity,
    size_t alignmentSamples) const
{
    if (imuData.size() < alignmentSamples)
    {
        std::cerr << "Not enough IMU data for alignment ("
                  << imuData.size() << " < " << alignmentSamples << ").\n";
        return Eigen::Vector3d::Zero();
    }

    double sum_fx = 0.0, sum_fy = 0.0, sum_fz = 0.0;
    double sum_wx = 0.0, sum_wy = 0.0;

    // Average over the first N samples
    for (size_t i = 0; i < alignmentSamples; ++i)
    {
        const auto &r = imuData[i];
        Eigen::Vector3d f = calib.correctAccelerometer(
            {r.accel_x, r.accel_y, r.accel_z});
        Eigen::Vector3d w = calib.correctGyro(
            {r.gyro_x, r.gyro_y, r.gyro_z});
        sum_fx += f.x();
        sum_fy += f.y();
        sum_fz += f.z();
        sum_wx += w.x();
        sum_wy += w.y();
    }

    double N = double(alignmentSamples);
    double avg_fx = sum_fx / N;
    double avg_fy = sum_fy / N;
    double avg_fz = sum_fz / N;
    double avg_wx = sum_wx / N;
    double avg_wy = sum_wy / N;

    double sign_fz = (avg_fz >= 0.0 ? 1.0 : -1.0);
    double roll = -sign_fz * std::asin(avg_fx / normalGravity);
    double pitch = sign_fz * std::asin(avg_fy / normalGravity);
    double azimuth = std::atan2(-avg_wy, avg_wx); // matched to MATLAB

    return {roll, pitch, azimuth};
}

// Returns a 3x3 rotation matrix based on roll, pitch, and azimuth (in radians).
Eigen::Matrix3d INSMechanization::computeRotationMatrix(double roll, double pitch, double azimuth)
{
    // For brevity, define sX = sin(X), cX = cos(X)
    double sr = std::sin(roll);
    double cr = std::cos(roll);
    double sp = std::sin(pitch);
    double cp = std::cos(pitch);
    double sA = std::sin(azimuth);
    double cA = std::cos(azimuth);
    Eigen::Matrix3d R;
    R << cA * cr + sA * sp * sr, sA * cp, cA * sr - sA * sp * cr,
        -sA * cr + cA * sp * sr, cA * cp, -sA * sr - cA * sp * cr,
        -cp * sr, sp, cp * cr;
    return R;
}

Eigen::Vector4d INSMechanization::matrixToQuaternionSimple(const Eigen::Matrix3d &R)
{

    double q4 = 0.5 * sqrt(1 + R(0, 0) + R(1, 1) + R(2, 2));
    double q1 = 0.25 * (R(2, 1) - R(1, 2)) / q4;
    double q2 = 0.25 * (R(0, 2) - R(2, 0)) / q4;
    double q3 = 0.25 * (R(1, 0) - R(0, 1)) / q4;
    Eigen::Vector4d q(q1, q2, q3, q4);

    double sum_q = pow(q1, 2) + pow(q2, 2) + pow(q3, 2) + pow(q4, 2);
    if (abs(sum_q - 1) > 1e-8)
    {
        double delta = 1 - sum_q;
        q = q / sqrt(1 - delta);
    }

    return q;
}

Eigen::Vector3d INSMechanization::computeTransportRate(Eigen::Vector3d velocity,
                                                       double primeVertical,
                                                       double meridian,
                                                       double height,
                                                       double latitude)
{
    // Extract north and east components from the velocity vector.
    double Ve = velocity(0);
    double Vn = velocity(1);

    // Earth's rotation rate (rad/s)
    double omega_e = EARTH_ROTATION_RATE;

    // Compute the local-level transport rate vector using the provided parameters.
    // Note: Adjust these formulas if your convention differs (e.g., NED vs. ENU).
    Eigen::Vector3d omega_local;
    // East component:
    omega_local(0) = -Vn / (meridian + height);
    // North component:
    omega_local(1) = Ve / (primeVertical + height) + (omega_e * std::cos(latitude));
    // Up component:
    omega_local(2) = (Ve * std::tan(latitude)) / (primeVertical + height) + (omega_e * std::sin(latitude));

    return omega_local;
}

Eigen::Matrix3d INSMechanization::quaternionToRotationMatrix(Eigen::Vector4d q)
{
    Eigen::Matrix3d R;
    R(0, 0) = pow(q(0), 2) - pow(q(1), 2) - pow(q(2), 2) + pow(q(3), 2);
    R(0, 1) = 2 * (q(0) * q(1) - q(2) * q(3));
    R(0, 2) = 2 * (q(0) * q(2) + q(1) * q(3));

    R(1, 0) = 2 * (q(0) * q(1) + q(2) * q(3));
    R(2, 1) = -pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) + pow(q(3), 2);
    R(2, 2) = 2 * (q(1) * q(2) - q(0) * q(3));

    R(2, 0) = 2 * (q(0) * q(2) - q(1) * q(3));
    R(2, 1) = 2 * (q(1) * q(2) + q(0) * q(3));
    R(2, 2) = -pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2);

    return R;
}

Eigen::Vector4d INSMechanization::updateQuaternion(const Eigen::Vector4d q_old, const Eigen::Vector3d &angularVelocity)
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

    M(0, 1) = angularVelocity(2);
    M(0, 2) = -angularVelocity(1);
    M(0, 3) = angularVelocity(0);

    M(1, 0) = -angularVelocity(2);
    M(1, 2) = angularVelocity(0);
    M(1, 3) = angularVelocity(1);

    M(2, 0) = angularVelocity(1);
    M(2, 1) = -angularVelocity(0);
    M(2, 3) = angularVelocity(2);

    M(3, 0) = -angularVelocity(0);
    M(3, 1) = -angularVelocity(1);
    M(3, 2) = -angularVelocity(2);
    double delta_t = 1 / 64;

    Eigen::Vector4d q = q_old + (0.5 * M * q_old) * delta_t;
    // check if q equals 1
    double sum_q = pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2);
    if (sum_q != 1)
    {
        double delta = 1 - sum_q;
        // q = (q /sqrt(1 - delta));
        q(0) = q(0) / sqrt(1 - delta);
        q(1) = q(1) / sqrt(1 - delta);
        q(2) = q(2) / sqrt(1 - delta);
        q(3) = q(3) / sqrt(1 - delta);
    }
    return q;
}

void INSMechanization::run()
{
    std::cout << "Running INS Mechanization..." << std::endl;
    std::cout << "Initial Latitude (rad): " << initLatitude << std::endl;
    std::cout << "Initial Longitude (rad): " << initLongitude << std::endl;
    std::cout << "Initial Height (m): " << initHeight << std::endl;
    std::cout << "Number of IMU records: " << imuData.size() << std::endl;

    std::ofstream csvFile("../results/position_output.csv");
    csvFile << "time,latitude_deg,longitude_deg,height_m\n";

    // --- Create and Configure a Calibration Object for the Alignment Window ---
    IMUCalibration calib;

    // Example calibration values
    double gyroBias_degHour = 0.1; // Gyro bias in deg/hr
    double accelBias_microG = 3;   // Accelerometer bias in micro-g

    // Convert bias values using functions from Utilities.h
    double gyroBias_radPerSec = gyroBiasDegHrToRadSec(gyroBias_degHour);
    double accelBias_mPerSec2 = accelBiasMicrogToMPerSec2(accelBias_microG);

    // Set accelerometer calibration parameters
    Eigen::Vector3d accelBias(accelBias_mPerSec2, accelBias_mPerSec2, accelBias_mPerSec2);
    Eigen::Vector3d accelScale(1.0, 1.0, 1.0);            // Assume nominal scale factors
    Eigen::Vector3d accelNonOrthogonality(0.0, 0.0, 0.0); // Assume ideal alignment
    calib.setAccelBias(accelBias);
    calib.setAccelScale(accelScale);
    calib.setAccelNonOrthogonality(accelNonOrthogonality);

    // Set gyroscope calibration parameters
    Eigen::Vector3d gyroBias(gyroBias_radPerSec, gyroBias_radPerSec, gyroBias_radPerSec);
    Eigen::Vector3d gyroScale(1.0, 1.0, 1.0);
    Eigen::Vector3d gyroNonOrthogonality(0.0, 0.0, 0.0);
    calib.setGyroBias(gyroBias);
    calib.setGyroScale(gyroScale);
    calib.setGyroNonOrthogonality(gyroNonOrthogonality);
    // ---------------------------------------------------------------------------
    // Compute and print Earth model parameters
    // ---------------------------------------------------------------------------
    double normalGravity = computeNormalGravity(initLatitude, initHeight);
    double radiusMeridian = computeRadiusMeridian(initLatitude);
    double radiusPrimeVertical = computeRadiusPrimeVertical(initLatitude);

    // Initial Aligmnent
    size_t alignmentSamples = 4000;
    Eigen::Vector3d alignment = computeInitialAlignment(calib, normalGravity, alignmentSamples);
    double roll_rad = alignment[0];
    double pitch_rad = alignment[1];
    double azimuth_rad = alignment[2];
    Eigen::Matrix3d R = computeRotationMatrix(roll_rad, pitch_rad, azimuth_rad);
    std::cout << R << std::endl;
    Eigen::Vector4d q = matrixToQuaternionSimple(R);

    R = quaternionToRotationMatrix(q);

    Eigen::Vector3d V_l = initVelocity;
    double latitude = initLatitude;
    double longitude = initLongitude;
    double height = initHeight;
    double omega_e = EARTH_ROTATION_RATE;

    Eigen::Vector3d prevDeltaVl = Eigen::Vector3d::Zero();

    double delta_t = 1.0 / 64.0;
    for (auto &record : imuData)
    {

        Eigen::Vector3d rawGyro(record.gyro_x, record.gyro_y, record.gyro_z);
        // Eigen::Vector3d correctedGyro = calib.correctGyro(rawGyro);

        Eigen::Vector3d rawAccel(record.accel_x, record.accel_y, record.accel_z);
        // Eigen::Vector3d correctedAccel = calib.correctAccelerometer(rawAccel);

        Eigen::Vector3d transportRate = computeTransportRate(V_l, radiusPrimeVertical, radiusMeridian, height, latitude);

        Eigen::Vector3d transportRate_b = R.transpose() * transportRate;

        // Eigen::Vector3d transportIncrement = transportRate_b * delta_t;

        Eigen::Vector3d theta_lib = transportRate_b * delta_t;
        Eigen::Vector3d theta_bib = rawGyro - theta_lib;
        q = updateQuaternion(q, theta_bib);
        R = quaternionToRotationMatrix(q);

        // Ω_ie^l  (earth rotation seen in local-level frame)
        Eigen::Matrix3d Oi;
        Oi << 0, -omega_e * std::sin(latitude), omega_e * std::cos(latitude),
            omega_e * std::sin(latitude), 0, 0,
            -omega_e * std::cos(latitude), 0, 0;

        // Ω_el^l  (transport rate)
        Eigen::Matrix3d Ot;
        Ot << 0, -V_l(0) * std::tan(latitude) / (radiusPrimeVertical + height), V_l(0) / (radiusPrimeVertical + height),
            V_l(0) * std::tan(latitude) / (radiusPrimeVertical + height), 0, V_l(1) / (radiusMeridian + height),
            -V_l(0) / (radiusPrimeVertical + height), -V_l(1) / (radiusMeridian + height), 0;

        Eigen::Vector3d g_l(0.0, 0.0, -normalGravity);

        Eigen::Vector3d deltaVb = rawAccel * delta_t;
        Eigen::Vector3d deltaVl = R * deltaVb - (2.0 * Oi + Ot) * V_l * delta_t + g_l * delta_t;

        Eigen::Vector3d V_l_prev = V_l;
        V_l = V_l + 0.5 * (prevDeltaVl + deltaVl);
        prevDeltaVl = deltaVl;

        double Vup_prev = V_l_prev(2); // Previous V^u
        double Vup_curr = V_l(2);      // Current V^u
        height = height + 0.5 * (Vup_prev + Vup_curr) * delta_t;

        double Vn_prev = V_l_prev(1); // Previous V^n
        double Vn_curr = V_l(1);      // Current V^n
        // double lat_prev = latitude;
        latitude = latitude + 0.5 * ((Vn_prev + Vn_curr) / (radiusMeridian + height)) * delta_t;

        double Ve_prev = V_l_prev(0);
        double Ve_curr = V_l(0);
        double cosLat = std::cos(latitude);
        if (std::abs(cosLat) < 1e-6)
            cosLat = 1e-6; // avoid near-zero division

        longitude += 0.5 * ((Ve_prev + Ve_curr) / ((radiusPrimeVertical + height) * cosLat)) * delta_t;
        // {
        //     std::cout << "Lat: " << latitude * RAD_TO_DEG << " degrees, Lon: " << longitude * RAD_TO_DEG << " degrees\n";
        // }

        radiusMeridian = computeRadiusMeridian(latitude);
        radiusPrimeVertical = computeRadiusPrimeVertical(latitude);
        normalGravity = computeNormalGravity(latitude, height);

        csvFile << record.time << ","
                << latitude * RAD_TO_DEG << ","
                << longitude * RAD_TO_DEG << ","
                << height << "\n";
    }
    csvFile.close();
}

void INSMechanization::printResulst() const
{
}
