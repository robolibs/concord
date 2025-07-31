#pragma once

#include "constants.hpp"
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace concord {
    using namespace concord::constants;

    // Function to determine the UTM zone for a given longitude
    inline int get_utm_zone(double longitude) { return static_cast<int>(std::floor((longitude + 180.0) / 6.0) + 1); }

    // Function to determine if a given latitude is in the northern hemisphere
    inline bool is_northern_hemisphere(double latitude) { return latitude >= 0.0; }

    // Function to convert latitude/longitude (WGS-84) to UTM coordinates
    inline std::tuple<double, double, int, bool> wgs_to_utm(double latitude, double longitude) {
        // Check latitude bounds
        if (latitude < -80.0 || latitude > 84.0) {
            throw std::out_of_range("Latitude out of UTM bounds (-80 to 84 degrees).");
        }

        // Convert latitude and longitude from degrees to radians
        double lat_rad = latitude * M_PI / 180.0;
        double lon_rad = longitude * M_PI / 180.0;

        // Get the UTM zone
        int zone = get_utm_zone(longitude);

        // Calculate central meridian for the zone
        double lon0 = ((zone - 1) * 6 - 180 + 3) * M_PI / 180.0; // Central meridian

        // Compute trigonometric functions
        double sinLat = std::sin(lat_rad);
        double cosLat = std::cos(lat_rad);
        double tanLat = std::tan(lat_rad);

        // Radius of curvature in the prime vertical
        double N = a / std::sqrt(1 - e2 * sinLat * sinLat);
        double T = tanLat * tanLat;
        double C = ep2 * cosLat * cosLat;
        double A = cosLat * (lon_rad - lon0);

        // Meridional arc
        double M =
            a * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * lat_rad -
                 (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * std::sin(2 * lat_rad) +
                 (15 * e4 / 256 + 45 * e6 / 1024) * std::sin(4 * lat_rad) - (35 * e6 / 3072) * std::sin(6 * lat_rad));

        // Calculate UTM Easting (x) and Northing (y)
        double easting =
            k0 * N *
                (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * A * A * A * A * A / 120) +
            500000.0; // Add the false easting

        double northing = k0 * (M + N * tanLat *
                                        (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                                         (61 - 58 * T + T * T + 600 * C - 330 * ep2) * A * A * A * A * A * A / 720));

        // Adjust for southern hemisphere
        if (!is_northern_hemisphere(latitude)) {
            northing += 10000000.0; // False northing for southern hemisphere
        }

        // Return UTM coordinates: easting, northing, UTM zone, and hemisphere (true for northern)
        return std::make_tuple(easting, northing, zone, is_northern_hemisphere(latitude));
    }

    // Function to convert UTM to WGS-84 (latitude, longitude)
    inline std::tuple<double, double> utm_to_wgs(double easting, double northing, int zone,
                                                 bool is_northern_hemisphere = true) {
        // Adjust for the southern hemisphere
        if (!is_northern_hemisphere) {
            northing -= 10000000.0;
        }

        // Central meridian of the UTM zone
        double lon0 = ((zone - 1) * 6 - 180 + 3) * M_PI / 180.0;

        // Meridional arc
        double M = northing / k0;
        double mu = M / (a * (1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256));

        // Footprint latitude
        double e1 = (1 - std::sqrt(1 - e2)) / (1 + std::sqrt(1 - e2));
        double phi1 = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * std::sin(2 * mu) +
                      (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * std::sin(4 * mu) +
                      (151 * e1 * e1 * e1 / 96) * std::sin(6 * mu) +
                      (1097 * e1 * e1 * e1 * e1 / 512) * std::sin(8 * mu);

        // Compute trigonometric functions
        double sinPhi1 = std::sin(phi1);
        double cosPhi1 = std::cos(phi1);
        double tanPhi1 = std::tan(phi1);

        // Radius of curvature in the prime vertical
        double N1 = a / std::sqrt(1 - e2 * sinPhi1 * sinPhi1);
        double T1 = tanPhi1 * tanPhi1;
        double C1 = ep2 * cosPhi1 * cosPhi1;
        double R1 = a * (1 - e2) / std::pow(1 - e2 * sinPhi1 * sinPhi1, 1.5);
        double D = (easting - 500000.0) / (N1 * k0);

        // Calculate latitude
        double latitude = phi1 - (N1 * tanPhi1 / R1) *
                                     (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * ep2) * D * D * D * D / 24 +
                                      (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * ep2 - 3 * C1 * C1) * D * D * D *
                                          D * D * D / 720);

        // Calculate longitude
        double longitude =
            lon0 + (D - (1 + 2 * T1 + C1) * D * D * D / 6 +
                    (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * ep2 + 24 * T1 * T1) * D * D * D * D * D / 120) /
                       cosPhi1;

        // Convert latitude and longitude to degrees
        latitude = latitude * 180.0 / M_PI;
        longitude = longitude * 180.0 / M_PI;

        // Return the WGS-84 coordinates (latitude, longitude)
        return std::make_tuple(latitude, longitude);
    }
} // namespace concord
