#pragma once

#include "constants.hpp"
#include <cmath>
#include <tuple>

namespace concord {

    // High-precision function to convert GPS (lat, lon, alt) to ECEF coordinates
    inline std::tuple<double, double, double> gps_to_ecef(double latitude, double longitude, double altitude) {
        // Convert latitude and longitude to radians with high precision
        const double lat_rad = latitude * constants::DEG_TO_RAD;
        const double lon_rad = longitude * constants::DEG_TO_RAD;

        const double cos_lat = std::cos(lat_rad);
        const double sin_lat = std::sin(lat_rad);
        const double cos_lon = std::cos(lon_rad);
        const double sin_lon = std::sin(lon_rad);

        // Calculate the radius of curvature in the prime vertical with high precision
        const double sin_lat_sq = sin_lat * sin_lat;
        const double N =
            constants::WGS84_EQUATORIAL_RADIUS / std::sqrt(1.0 - constants::WGS84_ECCENTRICITY_SQUARED * sin_lat_sq);

        // Calculate ECEF coordinates
        const double x = (N + altitude) * cos_lat * cos_lon;
        const double y = (N + altitude) * cos_lat * sin_lon;
        const double z = (N * (1.0 - constants::WGS84_ECCENTRICITY_SQUARED) + altitude) * sin_lat;

        return std::make_tuple(x, y, z);
    }

    // High-precision function to convert ECEF coordinates to ENU with respect to a datum
    inline std::tuple<double, double, double> ecef_to_enu(std::tuple<double, double, double> ecef,
                                                          std::tuple<double, double, double> datum) {
        const auto [x, y, z] = ecef;
        const auto [lat_ref, lon_ref, alt_ref] = datum;

        // Convert the reference latitude and longitude to radians
        const double lat_ref_rad = lat_ref * constants::DEG_TO_RAD;
        const double lon_ref_rad = lon_ref * constants::DEG_TO_RAD;

        const double cos_lat_ref = std::cos(lat_ref_rad);
        const double sin_lat_ref = std::sin(lat_ref_rad);
        const double cos_lon_ref = std::cos(lon_ref_rad);
        const double sin_lon_ref = std::sin(lon_ref_rad);

        // Prime vertical radius of curvature at the reference point
        const double sin_lat_ref_sq = sin_lat_ref * sin_lat_ref;
        const double N_ref = constants::WGS84_EQUATORIAL_RADIUS /
                             std::sqrt(1.0 - constants::WGS84_ECCENTRICITY_SQUARED * sin_lat_ref_sq);

        // Calculate the reference ECEF coordinates (datum)
        const double x0 = (N_ref + alt_ref) * cos_lat_ref * cos_lon_ref;
        const double y0 = (N_ref + alt_ref) * cos_lat_ref * sin_lon_ref;
        const double z0 = (N_ref * (1.0 - constants::WGS84_ECCENTRICITY_SQUARED) + alt_ref) * sin_lat_ref;

        // Calculate the differences between the ECEF coordinates and the reference ECEF coordinates
        const double dx = x - x0;
        const double dy = y - y0;
        const double dz = z - z0;

        // Calculate the ENU coordinates using standard rotation matrix
        const double x_east = -sin_lon_ref * dx + cos_lon_ref * dy;
        const double y_north = -cos_lon_ref * sin_lat_ref * dx - sin_lat_ref * sin_lon_ref * dy + cos_lat_ref * dz;
        const double z_up = cos_lat_ref * cos_lon_ref * dx + cos_lat_ref * sin_lon_ref * dy + sin_lat_ref * dz;

        return std::make_tuple(x_east, y_north, z_up);
    }

    // High-precision function to convert ENU to ECEF using the inverse transformation matrix
    inline std::tuple<double, double, double> enu_to_ecef(std::tuple<double, double, double> enu,
                                                          std::tuple<double, double, double> datum) {
        const auto [x_east, y_north, z_up] = enu;
        const auto [lat_ref, lon_ref, alt_ref] = datum;

        // Convert the reference latitude and longitude to radians
        const double lat_ref_rad = lat_ref * constants::DEG_TO_RAD;
        const double lon_ref_rad = lon_ref * constants::DEG_TO_RAD;

        const double cos_lat_ref = std::cos(lat_ref_rad);
        const double sin_lat_ref = std::sin(lat_ref_rad);
        const double cos_lon_ref = std::cos(lon_ref_rad);
        const double sin_lon_ref = std::sin(lon_ref_rad);

        // Prime vertical radius of curvature at the reference point
        const double sin_lat_ref_sq = sin_lat_ref * sin_lat_ref;
        const double N_ref = constants::WGS84_EQUATORIAL_RADIUS /
                             std::sqrt(1.0 - constants::WGS84_ECCENTRICITY_SQUARED * sin_lat_ref_sq);

        // Calculate the reference ECEF coordinates (datum)
        const double x0 = (N_ref + alt_ref) * cos_lat_ref * cos_lon_ref;
        const double y0 = (N_ref + alt_ref) * cos_lat_ref * sin_lon_ref;
        const double z0 = (N_ref * (1.0 - constants::WGS84_ECCENTRICITY_SQUARED) + alt_ref) * sin_lat_ref;

        // Apply inverse transformation matrix (transpose of ENU to ECEF rotation matrix)
        const double dx =
            -sin_lon_ref * x_east - cos_lon_ref * sin_lat_ref * y_north + cos_lat_ref * cos_lon_ref * z_up;
        const double dy = cos_lon_ref * x_east - sin_lat_ref * sin_lon_ref * y_north + cos_lat_ref * sin_lon_ref * z_up;
        const double dz = cos_lat_ref * y_north + sin_lat_ref * z_up;

        // Calculate ECEF coordinates
        const double x = x0 + dx;
        const double y = y0 + dy;
        const double z = z0 + dz;

        return std::make_tuple(x, y, z);
    }

    // High-precision ECEF to GPS conversion using improved iterative method
    inline std::tuple<double, double, double> ecef_to_gps(double x, double y, double z) {
        const double a = constants::WGS84_EQUATORIAL_RADIUS;
        const double e2 = constants::WGS84_ECCENTRICITY_SQUARED;
        const double eps = 1e-15; // Higher precision convergence threshold

        // Longitude calculation (exact)
        const double longitude = std::atan2(y, x) * constants::RAD_TO_DEG;

        // Distance from z-axis
        const double p = std::sqrt(x * x + y * y);

        // Initial latitude guess using improved method
        double latitude = std::atan2(z, p * (1.0 - e2));
        double N, altitude;
        double lat_old;

        // Iterative refinement with higher precision
        int max_iterations = 20;
        for (int i = 0; i < max_iterations; ++i) {
            lat_old = latitude;
            const double sin_lat = std::sin(latitude);
            const double cos_lat = std::cos(latitude);

            N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
            altitude = (p / cos_lat) - N;

            // More accurate latitude update
            latitude = std::atan2(z, p * (1.0 - e2 * N / (N + altitude)));

            if (std::abs(latitude - lat_old) < eps) {
                break;
            }
        }

        return std::make_tuple(latitude * constants::RAD_TO_DEG, longitude, altitude);
    }

    // High-precision function to convert GPS to ENU directly
    inline std::tuple<double, double, double> gps_to_enu(double latitude, double longitude, double altitude,
                                                         double lat_ref, double lon_ref, double alt_ref) {
        // Convert GPS coordinates to ECEF
        auto ecef = gps_to_ecef(latitude, longitude, altitude);
        // Convert ECEF to ENU
        return ecef_to_enu(ecef, std::make_tuple(lat_ref, lon_ref, alt_ref));
    }

    // High-precision function to convert ENU to GPS
    inline std::tuple<double, double, double> enu_to_gps(double x_east, double y_north, double z_up, double lat_ref,
                                                         double lon_ref, double alt_ref) {
        // Convert ENU to ECEF
        auto ecef = enu_to_ecef(std::make_tuple(x_east, y_north, z_up), std::make_tuple(lat_ref, lon_ref, alt_ref));
        // Convert ECEF to GPS
        return ecef_to_gps(std::get<0>(ecef), std::get<1>(ecef), std::get<2>(ecef));
    }

    // Convenience functions for small displacement calculations (centimeter precision)
    inline std::tuple<double, double, double> gps_displacement_to_enu(double lat_base, double lon_base, double alt_base,
                                                                      double lat_target, double lon_target,
                                                                      double alt_target) {
        return gps_to_enu(lat_target, lon_target, alt_target, lat_base, lon_base, alt_base);
    }

    inline std::tuple<double, double, double> enu_displacement_to_gps(double x_east, double y_north, double z_up,
                                                                      double lat_base, double lon_base,
                                                                      double alt_base) {
        return enu_to_gps(x_east, y_north, z_up, lat_base, lon_base, alt_base);
    }
} // namespace concord
