#include "EarthModel.h"
#include "Constants.h" // so we can use a1, a2, a3, etc.

double computeNormalGravity(double latitude, double height)
{
    // sin^2(phi) and sin^4(phi)
    double sinLat = sin(latitude);
    double sinLat2 = sinLat * sinLat;
    double sinLat4 = sinLat2 * sinLat2;
    double term1 = GRAVITY_A1 * (1.0 + GRAVITY_A2 * sinLat2 + GRAVITY_A3 * sinLat4);
    double term2 = (GRAVITY_A4 + GRAVITY_A5 * sinLat2) * height;
    double term3 = GRAVITY_A6 * (height * height);

    return  term1 + term2 + term3;
}

double computeRadiusMeridian(double latitude)
{
    // M = (a(1-e^2)) / (1 - e^2 sin^2(lat))^(3/2)
    // or incorporate height if needed
    double sinLat2 = sin(latitude) * sin(latitude);
    double denom = std::pow((1.0 - WGS84_ECCENTRICITY_SQUARED * sinLat2), 1.5);
    double numerator = WGS84_SEMI_MAJOR_AXIS * (1 - WGS84_ECCENTRICITY_SQUARED);
    return numerator / denom;
}

double computeRadiusPrimeVertical(double latitude)
{
    // N = a / sqrt(1 - e^2 sin^2(lat))
    double sinLat2 = sin(latitude) * sin(latitude);
    double denom = std::sqrt(1.0 - WGS84_ECCENTRICITY_SQUARED * sinLat2);
    return WGS84_SEMI_MAJOR_AXIS / denom;
}
