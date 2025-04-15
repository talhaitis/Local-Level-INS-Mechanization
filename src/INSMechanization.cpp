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
Eigen::Vector3d INSMechanization::computeInitialAlignment(const IMUCalibration &calib, double normalGravity, double alignmentWindow) const
{
    if (imuData.empty())
    {
        std::cerr << "No IMU data available for alignment." << std::endl;
        return Eigen::Vector3d::Zero();
    }

    double sum_fx = 0.0, sum_fy = 0.0, sum_fz = 0.0;
    double sum_wx = 0.0, sum_wy = 0.0;
    int count = 0;

    for (const auto &record : imuData)
    {
        if (record.time <= alignmentWindow)
        {
            // Apply calibration corrections for each measurement:
            Eigen::Vector3d rawAccel(record.accel_x, record.accel_y, record.accel_z);
            Eigen::Vector3d rawGyro(record.gyro_x, record.gyro_y, record.gyro_z);
            Eigen::Vector3d correctedAccel = calib.correctAccelerometer(rawAccel);
            Eigen::Vector3d correctedGyro = calib.correctGyro(rawGyro);

            // Accumulate the corrected values:
            sum_fx += correctedAccel.x();
            sum_fy += correctedAccel.y();
            sum_fz += correctedAccel.z();

            sum_wx += correctedGyro.x();
            sum_wy += correctedGyro.y();
            ++count;
        }
    }

    if (count == 0)
    {
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

    // |  cA * cp     +  sA * sp * sr    ,    sA * cp      -  cA * sp * sr   ,   sA * sp + cA * cp * sr  |
    // | -cA * sp     +  sA * sr * cp    ,    cA * cp      +  sA * sp * sr   ,  -sA * cp + cA * sp * sr   |
    // | -sr * cp     ,    -sr * sp      ,          cr * cp                         ...                 |
    //

    Eigen::Matrix3d R;
    R(0, 0) = cA * cr + sA * sp * sr; // cosA cosr + sinA sinp sinr
    R(0, 1) = sA * cp;                // sinA cosp
    R(0, 2) = cA * sr - sA * sp * cr; // cosA sinr - sinA sinp cosr

    R(1, 0) = -sA * cr + cA * sp * sr; // -sinA cosr + cosA sinp sinr
    R(1, 1) = cA * cp;                 // cosA cosp
    R(1, 2) = -sA * sr - cA * sp * cr; // -sinA sinr - cosA sinp cosr

    R(2, 0) = -cp * sr; // -cos p sin r
    R(2, 1) = sp;       // sinp
    R(2, 2) = cp * cr;  // cosp cosr

    return R;
}

Eigen::Vector4d INSMechanization::matrixToQuaternionSimple(const Eigen::Matrix3d &R)
{
    // For convenience, name the matrix elements with 1-based notation as your doc might,
    // but remember in code it's R(row, col) zero-based.
    double r11 = R(0, 0);
    double r12 = R(0, 1);
    double r13 = R(0, 2);
    double r21 = R(1, 0);
    double r22 = R(1, 1);
    double r23 = R(1, 2);
    double r31 = R(2, 0);
    double r32 = R(2, 1);
    double r33 = R(2, 2);

    double q4 = 0.5 * sqrt(1 + r11 + r22 + r33);
    double q3 = (0.25 * (r21 - r12)) / q4;
    double q2 = (0.25 * (r13 - r31)) / q4;
    double q1 = (0.25 * (r32 - r23)) / q4;
    double sum_q = pow(q1, 2) + pow(q2, 2) + pow(q3, 2) + pow(q4, 2);
    double delta = 1 - sum_q;
    printf("Sum q: %f\n", sum_q);
    printf("delta: %f\n", delta);
    Eigen::Vector4d q;
    // Here we choose to return [q1, q2, q3, q0].
    // Adjust the ordering if your document expects a different convention.
    q << q1, q2, q3, q4;

    return q;
}

Eigen::Vector3d INSMechanization::computeTransportRate(Eigen::Vector3d velocity,
                                                       double primeVertical,
                                                       double meridian,
                                                       double height,
                                                       double latitude)
{
    // Extract north and east components from the velocity vector.
    double Vn = velocity(0); // Northward velocity
    double Ve = velocity(1); // Eastward velocity

    // Earth's rotation rate (rad/s)
    double omega_e = EARTH_ROTATION_RATE;

    // Compute the local-level transport rate vector using the provided parameters.
    // Note: Adjust these formulas if your convention differs (e.g., NED vs. ENU).
    Eigen::Vector3d omega_local;
    // North component:
    omega_local(0) = -Vn / (meridian + height);
    // East component:
    omega_local(1) = Ve / (primeVertical + height) + omega_e * std::cos(latitude);
    // Down component:
    omega_local(2) = (Ve * std::tan(latitude)) / (primeVertical + height) + omega_e * std::sin(latitude);

    return omega_local;
}

Eigen::Matrix3d INSMechanization::quaternionToRotationMatrix(Eigen::Vector4d q){
    Eigen::Matrix3d R;
    R (0 , 0) = pow(q(0), 2) - pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2);
}

Eigen::Vector4d INSMechanization::updateQuaternion(const Eigen::Vector4d &q_old, const Eigen::Vector3d &angularVelocity)
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

    M(0, 1) = angularVelocity(2);
    M(0, 2) = -angularVelocity(1);
    M(0, 3) = angularVelocity(0);

    M(1, 0) = -angularVelocity(1);
    M(1, 2) = angularVelocity(0);
    M(1, 3) = angularVelocity(1);

    M(2, 0) = angularVelocity(1);
    M(2, 1) = -angularVelocity(0);
    M(2, 3) = angularVelocity(2);

    M(3, 0) = -angularVelocity(0);
    M(3, 1) = -angularVelocity(1);
    M(3, 2) = -angularVelocity(2);

    Eigen::Vector4d q = q_old + 0.5 * M * q_old;
    // check if q equals 1 
    double sum_q = pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2);
    if(sum_q != 1){
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

    std::cout << "Computed Normal Gravity at latitude " << initLatitude
              << " and height " << initHeight << " is: " << normalGravity << " m/s²" << std::endl;
    std::cout << "Radius of Curvature (Meridian) at latitude " << initLatitude
              << " is: " << radiusMeridian << " m" << std::endl;
    std::cout << "Radius of Curvature (Prime Vertical) at latitude " << initLatitude
              << " is: " << radiusPrimeVertical << " m" << std::endl;

    // --- Compute the Initial Alignment over the first 120 seconds ---
    // This function applies the calibration to each measurement within the window,
    // averages the calibrated values, and computes the roll, pitch, and azimuth.
    double alignmentWindow = 120.0;
    Eigen::Vector3d alignment = computeInitialAlignment(calib, normalGravity, alignmentWindow);
    double roll_rad = alignment[0];
    double pitch_rad = alignment[1];
    double azimuth_rad = alignment[2];

    // Convert the angles to degrees for readability.
    double roll_deg = roll_rad * 180.0 / M_PI;
    double pitch_deg = pitch_rad * 180.0 / M_PI;
    double azimuth_deg = azimuth_rad * 180.0 / M_PI;

    std::cout << "Initial Alignment (from first " << alignmentWindow << " sec of calibrated data):" << std::endl;
    std::cout << "  Roll: " << roll_deg << " deg" << std::endl;
    std::cout << "  Pitch: " << pitch_deg << " deg" << std::endl;
    std::cout << "  Azimuth: " << azimuth_deg << " deg" << std::endl;

    // --- Compute the Initial Rotation Matrix from the Alignment Angles ---
    // Assume computeRotationMatrix is implemented (e.g., in a helper or EarthModel file)
    Eigen::Matrix3d R = computeRotationMatrix(roll_rad, pitch_rad, azimuth_rad);
    std::cout << "Initial Rotation Matrix R^b_l:" << std::endl;
    std::cout << R << std::endl;
    Eigen::Vector4d q = matrixToQuaternionSimple(R);

    // q[0], q[1], q[2], q[3] correspond to (q0, q1, q2, q3) from your doc
    std::cout << "Quaternion: " << q.transpose() << std::endl;

    Eigen::Vector3d transportRate = computeTransportRate(initVelocity, radiusPrimeVertical, radiusMeridian, initHeight, initLatitude);

    std::cout << "Transport Rate:" << std::endl;
    std::cout << "  North: " << transportRate(0) << " rad/s" << std::endl;
    std::cout << "  East: " << transportRate(1) << " rad/s" << std::endl;
    std::cout << "  Down: " << transportRate(2) << " rad/s" << std::endl;

    Eigen::Vector3d transportRate_b = R.transpose() * transportRate;

    std::cout << "Transport Rate in bodyframe:" << std::endl;
    std::cout << "  North: " << transportRate_b(0) << " rad/s" << std::endl;
    std::cout << "  East: " << transportRate_b(1) << " rad/s" << std::endl;
    std::cout << "  Down: " << transportRate_b(2) << " rad/s" << std::endl;

    double delta_t = 1.0 / 64.0;
    for (auto &record : imuData)
    {
        // If the data time is beyond the alignment window, we can do the compensation here
        if (record.time > alignmentWindow)
        {
            Eigen::Vector3d rawGyro(record.gyro_x, record.gyro_y, record.gyro_z);
            Eigen::Vector3d correctedGyro = calib.correctGyro(rawGyro);

            // Eigen::Vector3d transportIncrement = transportRate_b * delta_t;

            Eigen::Vector3d transportIncrement = transportRate_b * delta_t;
            Eigen::Vector3d compensaedAngularVelocity = correctedGyro - transportIncrement;
            q = updateQuaternion(q, compensaedAngularVelocity);
            if (record.time < alignmentWindow + 1.0)
            {
                std::cout << "Time: " << record.time << " s, Updated Quaternion: " << q.transpose() << std::endl;
            }
        }
    }
}

void INSMechanization::printResulst() const
{
}
