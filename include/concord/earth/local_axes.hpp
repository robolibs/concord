#pragma once

#include <cmath>
#include <datapod/datapod.hpp>
#include <optinum/lina/basic/matmul.hpp>
#include <optinum/lina/basic/transpose.hpp>
#include <optinum/simd/matrix.hpp>
#include <optinum/simd/vector.hpp>

#include "wgs84.hpp"

namespace concord::earth {

    namespace on = ::optinum;

    // Type aliases for SIMD-accelerated matrix operations
    using Matrix3d = on::simd::Matrix<double, 3, 3>;
    using Vector3d = on::simd::Vector<double, 3>;

    /**
     * @brief Matrix-vector multiplication using optinum SIMD
     * @param R 3x3 rotation matrix
     * @param v 3D point (dp::Point)
     * @return Rotated point as dp::Point
     */
    inline dp::Point mat_mul(const Matrix3d &R, const dp::Point &v) {
        // Convert dp::Point to Vector3d, multiply, convert back
        Vector3d vec{v.x, v.y, v.z};
        Vector3d result = R * vec;
        return dp::Point{result[0], result[1], result[2]};
    }

    /**
     * @brief Matrix-vector multiplication (overload for dp::mat::matrix3x3d)
     *
     * For backward compatibility with code using dp::mat::matrix3x3d directly.
     */
    inline dp::Point mat_mul(const dp::mat::matrix3x3d &R, const dp::Point &v) {
        // Wrap in Matrix3d and use SIMD version
        Matrix3d mat{R};
        return mat_mul(mat, v);
    }

    /**
     * @brief Matrix transpose using optinum SIMD
     * @param R 3x3 matrix
     * @return Transposed matrix
     */
    inline Matrix3d transpose(const Matrix3d &R) { return on::lina::transpose(R); }

    /**
     * @brief Matrix transpose (overload for dp::mat::matrix3x3d)
     *
     * For backward compatibility with code using dp::mat::matrix3x3d directly.
     */
    inline dp::mat::matrix3x3d transpose(const dp::mat::matrix3x3d &R) {
        Matrix3d mat{R};
        Matrix3d result = on::lina::transpose(mat);
        return result.pod();
    }

    /**
     * @brief Rotation matrix from ECEF to ENU frame
     *
     * Computes the rotation matrix that transforms vectors from
     * Earth-Centered Earth-Fixed (ECEF) coordinates to local
     * East-North-Up (ENU) coordinates at the given latitude/longitude.
     *
     * @param lat_rad Latitude in radians
     * @param lon_rad Longitude in radians
     * @return 3x3 rotation matrix (SIMD-accelerated)
     */
    inline Matrix3d R_enu_from_ecf(double lat_rad, double lon_rad) {
        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        // ENU rotation matrix (row-major initialization, stored column-major)
        // Row 0: East direction
        // Row 1: North direction
        // Row 2: Up direction
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
     * @return 3x3 rotation matrix (SIMD-accelerated)
     */
    inline Matrix3d R_ned_from_ecf(double lat_rad, double lon_rad) {
        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        // NED rotation matrix
        // Row 0: North direction
        // Row 1: East direction
        // Row 2: Down direction
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
