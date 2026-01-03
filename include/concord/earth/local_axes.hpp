#pragma once

#include <cmath>
#include <datapod/matrix/matrix.hpp>
#include <datapod/matrix/vector.hpp>
#include <optinum/simd/matrix.hpp>

#include "wgs84.hpp"

namespace concord::earth {

    namespace dp = ::datapod;

    // Type aliases
    // Both Matrix and Vector use datapod types (own their data)
    // Use optinum::simd::Matrix as a view when needed for matmul
    using Matrix3d = dp::mat::Matrix<double, 3, 3>;
    using Vector3d = dp::mat::Vector<double, 3>;

    /**
     * @brief Rotation matrix from ECEF to ENU frame
     *
     * Computes the rotation matrix that transforms vectors from
     * Earth-Centered Earth-Fixed (ECEF) coordinates to local
     * East-North-Up (ENU) coordinates at the given latitude/longitude.
     *
     * @param lat_rad Latitude in radians
     * @param lon_rad Longitude in radians
     * @return 3x3 rotation matrix
     */
    inline Matrix3d R_enu_from_ecf(double lat_rad, double lon_rad) {
        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        Matrix3d R{};
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

    /**
     * @brief Rotation matrix from ECEF to NED frame
     *
     * Computes the rotation matrix that transforms vectors from
     * Earth-Centered Earth-Fixed (ECEF) coordinates to local
     * North-East-Down (NED) coordinates at the given latitude/longitude.
     *
     * @param lat_rad Latitude in radians
     * @param lon_rad Longitude in radians
     * @return 3x3 rotation matrix
     */
    inline Matrix3d R_ned_from_ecf(double lat_rad, double lon_rad) {
        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        Matrix3d R{};
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
