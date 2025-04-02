#ifndef CONSTANTS_H
#define CONSTANTS_H

#include<cmath>

// Initial coordinates provided in the document 

const double INIT_LAT_DEG = 51.07995352;
const double INIT_LONG_DEG  = -114.13371127;
const double INIT_HEIGHT = 1118.502;

// Earth model parameters (WGS84)
const double WGS84_SEMI_MAJOR_AXIS = 6378137.0;          // in meters
const double WGS84_ECCENTRICITY_SQUARED = 6.69438e-3;     // unitless
const double EARTH_ROTATION_RATE = 7.292115147e-5;        // in rad/s

// Gravity model coefficients
const double GRAVITY_A1 = 9.7803267715;
const double GRAVITY_A2 = 0.0052790414;
const double GRAVITY_A3 = 0.0000232718;
const double GRAVITY_A4 = -0.000003087691089;
const double GRAVITY_A5 = 0.000000004397731;
const double GRAVITY_A6 = 0.000000000000721;

// Conversion factors
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

#endif // CONSTANTS_H
