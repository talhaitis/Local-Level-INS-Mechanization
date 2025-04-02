#ifndef EARTH_MODEL_H
#define EARTH_MODEL_H

// Forward declarations or includes
#include <cmath>

// A function to compute normal gravity
double computeNormalGravity(double latitude, double height);

// A function to compute the prime vertical radius (N) and meridian radius (M)
double computeRadiusMeridian(double latitude);
double computeRadiusPrimeVertical(double latitude);

#endif // EARTH_MODEL_H
