#pragma once

#include <cmath>
#include <numbers>

namespace concord::earth::wgs84 {

    inline constexpr double a_m = 6378137.0;         // semi-major axis (m)
    inline constexpr double f = 1.0 / 298.257223563; // flattening
    inline constexpr double b_m = a_m * (1.0 - f);   // semi-minor axis (m)
    inline constexpr double e2 = f * (2.0 - f);      // first eccentricity squared
    inline constexpr double ep2 = e2 / (1.0 - e2);   // second eccentricity squared
    inline constexpr double e4 = e2 * e2;
    inline constexpr double e6 = e4 * e2;

    inline constexpr double k0 = 0.9996; // UTM scale factor

    inline constexpr double deg_to_rad = std::numbers::pi / 180.0;
    inline constexpr double rad_to_deg = 180.0 / std::numbers::pi;

    inline double N(double sin_lat) { return a_m / std::sqrt(1.0 - e2 * sin_lat * sin_lat); }

} // namespace concord::earth::wgs84
