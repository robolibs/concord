#pragma once

#include "types.hpp"
#include "wgs84.hpp"
#include <cmath>

#include <datapod/matrix/vector.hpp>
#include <optinum/opti/quasi_newton/gauss_newton.hpp>

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

        // Longitude calculation (exact)
        const double lon = std::atan2(y, x);

        // Distance from z-axis
        const double p = std::sqrt(x * x + y * y);

        // Initial latitude guess using improved method
        double lat = std::atan2(z, p * (1.0 - wgs84::e2));
        double N = 0.0;
        double alt = 0.0;

        // Iterative refinement with high precision (sub-cm accuracy)
        constexpr double eps = 1e-15;
        constexpr int max_iterations = 20;

        for (int i = 0; i < max_iterations; ++i) {
            const double lat_old = lat;
            const double sin_lat = std::sin(lat);
            const double cos_lat = std::cos(lat);

            N = wgs84::N(sin_lat);
            alt = (p / cos_lat) - N;

            // More accurate latitude update
            lat = std::atan2(z, p * (1.0 - wgs84::e2 * N / (N + alt)));

            if (std::abs(lat - lat_old) < eps) {
                break;
            }
        }

        return WGS{lat * wgs84::rad_to_deg, lon * wgs84::rad_to_deg, alt};
    }

    /**
     * @brief Convert ECEF coordinates to WGS84 geodetic coordinates using optimization
     *
     * This variant uses Gauss-Newton optimization for configurable precision.
     * It minimizes the residual between the target ECF and the ECF computed from
     * the current lat/lon/alt estimate.
     *
     * Use this when:
     * - You need configurable convergence tolerance
     * - You have noisy ECF measurements and want best-fit WGS coordinates
     * - You want to integrate with other optimization workflows
     *
     * For most cases, the standard to_wgs() is sufficient and faster.
     *
     * @param ecf ECEF coordinates (x, y, z in meters)
     * @param tolerance Convergence tolerance for the optimizer (default 1e-12)
     * @return WGS84 coordinates (latitude, longitude in degrees, altitude in meters)
     */
    inline WGS to_wgs_optimized(const ECF &ecf, double tolerance = 1e-12) {
        using Vec3 = datapod::mat::Vector<double, 3>;

        const double x = ecf.x;
        const double y = ecf.y;
        const double z = ecf.z;

        // Initial estimate using simple approximation
        const double lon_init = std::atan2(y, x);
        const double p = std::sqrt(x * x + y * y);
        const double lat_init = std::atan2(z, p * (1.0 - wgs84::e2));
        const double alt_init = p / std::cos(lat_init) - wgs84::N(std::sin(lat_init));

        // Initial parameter vector: [lat_rad, lon_rad, alt]
        Vec3 params{lat_init, lon_init, alt_init};

        // Residual function: computes difference between target ECF and computed ECF
        // Returns 3 residuals (one per ECF component)
        auto residual_func = [x, y, z](const Vec3 &p) {
            const double lat_rad = p[0];
            const double lon_rad = p[1];
            const double alt = p[2];

            const double sin_lat = std::sin(lat_rad);
            const double cos_lat = std::cos(lat_rad);
            const double sin_lon = std::sin(lon_rad);
            const double cos_lon = std::cos(lon_rad);

            const double N = wgs84::N(sin_lat);

            // Computed ECF from current estimate
            const double x_comp = (N + alt) * cos_lat * cos_lon;
            const double y_comp = (N + alt) * cos_lat * sin_lon;
            const double z_comp = (N * (1.0 - wgs84::e2) + alt) * sin_lat;

            // Return residual (difference from target)
            return Vec3{x_comp - x, y_comp - y, z_comp - z};
        };

        // Configure Gauss-Newton optimizer
        optinum::opti::GaussNewton<double> optimizer;
        optimizer.max_iterations = 50;
        optimizer.tolerance = tolerance;
        optimizer.min_step_norm = tolerance * 0.1;
        optimizer.min_gradient_norm = tolerance * 0.1;
        optimizer.jacobian_central_diff = true;

        // Run optimization
        auto result = optimizer.optimize(residual_func, params);

        // Extract optimized parameters
        const double lat_opt = result.x[0];
        const double lon_opt = result.x[1];
        const double alt_opt = result.x[2];

        return WGS{lat_opt * wgs84::rad_to_deg, lon_opt * wgs84::rad_to_deg, alt_opt};
    }

} // namespace concord::earth
