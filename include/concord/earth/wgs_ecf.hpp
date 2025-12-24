#pragma once

#include "types.hpp"
#include "wgs84.hpp"
#include <cmath>

namespace concord::earth {

    inline ECF to_ecf(const WGS &wgs) {
        const double lat_rad = wgs.lat_deg * wgs84::deg_to_rad;
        const double lon_rad = wgs.lon_deg * wgs84::deg_to_rad;

        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        const double N = wgs84::N(sin_lat);
        return ECF{dp::Point{(N + wgs.alt_m) * cos_lat * cos_lon, (N + wgs.alt_m) * cos_lat * sin_lon,
                             (N * (1.0 - wgs84::e2) + wgs.alt_m) * sin_lat}};
    }

    inline WGS to_wgs(const ECF &ecf) {
        const double x = ecf.p.x;
        const double y = ecf.p.y;
        const double z = ecf.p.z;

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
