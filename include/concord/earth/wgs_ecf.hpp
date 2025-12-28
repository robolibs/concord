#pragma once

#include "types.hpp"
#include "wgs84.hpp"
#include <cmath>

namespace concord::earth {

    /**
     * @brief Convert WGS84 geodetic coordinates to ECEF (Earth-Centered Earth-Fixed)
     * @param wgs WGS84 coordinates (latitude, longitude in degrees, altitude in meters)
     * @return ECF coordinates (x, y, z in meters)
     */
    inline ECF to_ecf(const WGS &wgs) {
        const double lat_rad = wgs.latitude * wgs84::deg_to_rad;
        const double lon_rad = wgs.longitude * wgs84::deg_to_rad;

        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        const double N = wgs84::N(sin_lat);
        const double alt = wgs.altitude;
        // ECF extends dp::Point, so we can construct directly with x,y,z
        return ECF{(N + alt) * cos_lat * cos_lon, (N + alt) * cos_lat * sin_lon,
                   (N * (1.0 - wgs84::e2) + alt) * sin_lat};
    }

    /**
     * @brief Convert WGS84 geodetic coordinates to ECEF (overload for dp::Geo)
     */
    inline ECF to_ecf(const dp::Geo &geo) { return to_ecf(WGS{geo}); }

    /**
     * @brief Convert ECEF coordinates to WGS84 geodetic coordinates
     * @param ecf ECEF coordinates (x, y, z in meters)
     * @return WGS84 coordinates (latitude, longitude in degrees, altitude in meters)
     */
    inline WGS to_wgs(const ECF &ecf) {
        // ECF extends dp::Point, so x/y/z are direct members
        const double x = ecf.x;
        const double y = ecf.y;
        const double z = ecf.z;

        const double lon = std::atan2(y, x);
        const double p = std::sqrt(x * x + y * y);

        double lat = std::atan2(z, p * (1.0 - wgs84::e2));
        double N = 0.0;
        double alt = 0.0;

        for (int i = 0; i < 10; ++i) {
            const double sin_lat = std::sin(lat);
            const double cos_lat = std::cos(lat);
            N = wgs84::N(sin_lat);
            alt = (p / cos_lat) - N;

            const double lat_new = std::atan2(z, p * (1.0 - wgs84::e2 * N / (N + alt)));
            if (std::abs(lat_new - lat) < 1e-12) {
                break;
            }
            lat = lat_new;
        }

        return WGS{lat * wgs84::rad_to_deg, lon * wgs84::rad_to_deg, alt};
    }

} // namespace concord::earth
