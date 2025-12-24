#pragma once

#include <datapod/datapod.hpp>

#include "wgs84.hpp"
#include <cmath>

namespace concord::earth {

    inline dp::Point mat_mul(const dp::mat::matrix3x3d &R, const dp::Point &v) {
        return dp::Point{R(0, 0) * v.x + R(0, 1) * v.y + R(0, 2) * v.z, R(1, 0) * v.x + R(1, 1) * v.y + R(1, 2) * v.z,
                         R(2, 0) * v.x + R(2, 1) * v.y + R(2, 2) * v.z};
    }

    inline dp::mat::matrix3x3d transpose(const dp::mat::matrix3x3d &R) {
        dp::mat::matrix3x3d Rt{};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Rt(i, j) = R(j, i);
            }
        }
        return Rt;
    }

    inline dp::mat::matrix3x3d R_enu_from_ecf(double lat_rad, double lon_rad) {
        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        dp::mat::matrix3x3d R{};
        R(0, 0) = -sin_lon;
        R(0, 1) = cos_lon;
        R(0, 2) = 0.0;

        R(1, 0) = -sin_lat * cos_lon;
        R(1, 1) = -sin_lat * sin_lon;
        R(1, 2) = cos_lat;

        R(2, 0) = cos_lat * cos_lon;
        R(2, 1) = cos_lat * sin_lon;
        R(2, 2) = sin_lat;

        return R;
    }

    inline dp::mat::matrix3x3d R_ned_from_ecf(double lat_rad, double lon_rad) {
        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        dp::mat::matrix3x3d R{};
        R(0, 0) = -sin_lat * cos_lon;
        R(0, 1) = -sin_lat * sin_lon;
        R(0, 2) = cos_lat;

        R(1, 0) = -sin_lon;
        R(1, 1) = cos_lon;
        R(1, 2) = 0.0;

        R(2, 0) = -cos_lat * cos_lon;
        R(2, 1) = -cos_lat * sin_lon;
        R(2, 2) = -sin_lat;

        return R;
    }

} // namespace concord::earth
