#pragma once

#include "../../math.hpp"
#include "../wgs_to_enu.hpp"
#include "datum.hpp"
#include <cmath>
#include <tuple>

namespace concord {

    struct ENU; // Forward declaration for conversion method

    struct WGS {
        double lat = 0.0;
        double lon = 0.0;
        double alt = 0.0;

        WGS(double lat_, double lon_, double alt_) : lat(lat_), lon(lon_), alt(alt_) {
            validation::validate_latitude(lat);
            validation::validate_longitude(lon);
            validation::validate_altitude(alt);
        }

        WGS() = default;
        inline operator Datum() const noexcept { return Datum{lat, lon, alt}; }
        inline bool is_set() const { return lat != 0.0 && lon != 0.0; }

        ENU toENU(const Datum &datum) const;

        // Mathematical operations (for small displacements)
        inline WGS operator+(const WGS &offset) const {
            return WGS{lat + offset.lat, lon + offset.lon, alt + offset.alt};
        }
        inline WGS operator-(const WGS &offset) const {
            return WGS{lat - offset.lat, lon - offset.lon, alt - offset.alt};
        }

        // Distance calculation using haversine formula
        inline double distance_to(const WGS &other) const {
            validation::validate_finite(lat, "latitude");
            validation::validate_finite(lon, "longitude");
            validation::validate_finite(other.lat, "other latitude");
            validation::validate_finite(other.lon, "other longitude");

            const double R = 6371000.0; // Earth radius in meters
            double dlat = (other.lat - lat) * M_PI / 180.0;
            double dlon = (other.lon - lon) * M_PI / 180.0;
            double a = std::sin(dlat / 2) * std::sin(dlat / 2) + std::cos(lat * M_PI / 180.0) *
                                                                     std::cos(other.lat * M_PI / 180.0) *
                                                                     std::sin(dlon / 2) * std::sin(dlon / 2);
            double c = 2 * safe_math::safe_asin(safe_math::safe_sqrt(a, "haversine"), "haversine");
            return R * c;
        }

        // Bearing calculation
        inline double bearing_to(const WGS &other) const {
            double dlon = (other.lon - lon) * M_PI / 180.0;
            double lat1_rad = lat * M_PI / 180.0;
            double lat2_rad = other.lat * M_PI / 180.0;

            double y = std::sin(dlon) * std::cos(lat2_rad);
            double x =
                std::cos(lat1_rad) * std::sin(lat2_rad) - std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon);
            double bearing = std::atan2(y, x) * 180.0 / M_PI;
            return std::fmod(bearing + 360.0, 360.0);
        }

        inline bool operator==(const WGS &other) const {
            return lat == other.lat && lon == other.lon && alt == other.alt;
        }

        inline bool operator!=(const WGS &other) const { return !(*this == other); }
    };

} // namespace concord
