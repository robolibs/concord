#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

namespace concord {
    namespace constants {
        // Earth model constants (WGS-84)
        constexpr double WGS84_EQUATORIAL_RADIUS = 6378137.0; // meters
        constexpr double WGS84_FLATTENING = 1.0 / 298.257223563;
        constexpr double WGS84_ECCENTRICITY_SQUARED = WGS84_FLATTENING * (2 - WGS84_FLATTENING);
        constexpr double WGS84_ECCENTRICITY_FOURTH = WGS84_ECCENTRICITY_SQUARED * WGS84_ECCENTRICITY_SQUARED;
        constexpr double WGS84_ECCENTRICITY_SIXTH = WGS84_ECCENTRICITY_FOURTH * WGS84_ECCENTRICITY_SQUARED;
        constexpr double WGS84_SECOND_ECCENTRICITY_SQUARED =
            WGS84_ECCENTRICITY_SQUARED / (1 - WGS84_ECCENTRICITY_SQUARED);

        // UTM projection constants
        constexpr double UTM_SCALE_FACTOR = 0.9996;

        // Mathematical constants
        constexpr double PI = M_PI;
        constexpr double PI_2 = M_PI_2;
        constexpr double PI_4 = M_PI_4;
        constexpr double TWO_PI = 2.0 * M_PI;
        constexpr double RAD_TO_DEG = 180.0 / M_PI;
        constexpr double DEG_TO_RAD = M_PI / 180.0;
    } // namespace constants

    // Legacy aliases for backward compatibility
    constexpr double R = constants::WGS84_EQUATORIAL_RADIUS;
    constexpr double a = constants::WGS84_EQUATORIAL_RADIUS;
    constexpr double f = constants::WGS84_FLATTENING;
    constexpr double e2 = constants::WGS84_ECCENTRICITY_SQUARED;
    constexpr double e4 = constants::WGS84_ECCENTRICITY_FOURTH;
    constexpr double e6 = constants::WGS84_ECCENTRICITY_SIXTH;
    constexpr double ep2 = constants::WGS84_SECOND_ECCENTRICITY_SQUARED;
    constexpr double k0 = constants::UTM_SCALE_FACTOR;
    // Note: PI is defined in math/types_math.hpp
} // namespace concord
