#pragma once

#include <datapod/datapod.hpp>
#include <ostream>
#include <tuple>

#include "wgs84.hpp"

namespace concord::earth {

    // Unit for angular values
    enum class AngleUnit { Deg, Rad };

    // ============================================================================
    // WGS - WGS84 geodetic coordinates
    // ============================================================================
    struct WGS {
        double lat_deg = 0.0;
        double lon_deg = 0.0;
        double alt_m = 0.0;

        // Constructors
        WGS() = default;
        WGS(double lat, double lon, double alt = 0.0) : lat_deg(lat), lon_deg(lon), alt_m(alt) {}

        // Factory from radians
        static WGS from_radians(double lat_rad, double lon_rad, double alt = 0.0) {
            return WGS{lat_rad * wgs84::rad_to_deg, lon_rad * wgs84::rad_to_deg, alt};
        }

        // Accessors with unit conversion
        double latitude(AngleUnit unit = AngleUnit::Deg) const {
            return (unit == AngleUnit::Rad) ? lat_deg * wgs84::deg_to_rad : lat_deg;
        }
        double longitude(AngleUnit unit = AngleUnit::Deg) const {
            return (unit == AngleUnit::Rad) ? lon_deg * wgs84::deg_to_rad : lon_deg;
        }
        double altitude() const { return alt_m; }

        // Mutable accessors (always degrees)
        double &latitude() { return lat_deg; }
        double &longitude() { return lon_deg; }
        double &altitude() { return alt_m; }

        bool is_set() const noexcept { return lat_deg != 0.0 || lon_deg != 0.0 || alt_m != 0.0; }

        auto members() noexcept { return std::tie(lat_deg, lon_deg, alt_m); }
        auto members() const noexcept { return std::tie(lat_deg, lon_deg, alt_m); }
    };

    inline std::ostream &operator<<(std::ostream &os, const WGS &w) {
        return os << "WGS{lat=" << w.lat_deg << "deg, lon=" << w.lon_deg << "deg, alt=" << w.alt_m << "m}";
    }

    // ============================================================================
    // ECF - Earth-Centered Fixed (ECEF) coordinates
    // ============================================================================
    struct ECF {
        dp::Point p{}; // x,y,z in meters

        ECF() = default;
        explicit ECF(const dp::Point &pt) : p(pt) {}
        ECF(double x, double y, double z) : p{x, y, z} {}

        // Accessors
        double &x() { return p.x; }
        double &y() { return p.y; }
        double &z() { return p.z; }
        double x() const { return p.x; }
        double y() const { return p.y; }
        double z() const { return p.z; }

        bool is_set() const noexcept { return p.is_set(); }

        auto members() noexcept { return std::tie(p); }
        auto members() const noexcept { return std::tie(p); }
    };

    inline std::ostream &operator<<(std::ostream &os, const ECF &e) {
        return os << "ECF{x=" << e.p.x << "m, y=" << e.p.y << "m, z=" << e.p.z << "m}";
    }

    // ============================================================================
    // UTM - Universal Transverse Mercator coordinates
    // ============================================================================
    struct UTM {
        double easting_m = 0.0;
        double northing_m = 0.0;
        double alt_m = 0.0;
        int zone = 0;
        bool north = true;

        UTM() = default;
        UTM(double easting, double northing, double alt, int z, bool n)
            : easting_m(easting), northing_m(northing), alt_m(alt), zone(z), north(n) {}

        // Accessors
        double &easting() { return easting_m; }
        double &northing() { return northing_m; }
        double &altitude() { return alt_m; }
        double easting() const { return easting_m; }
        double northing() const { return northing_m; }
        double altitude() const { return alt_m; }

        bool is_set() const noexcept { return easting_m != 0.0 || northing_m != 0.0; }

        auto members() noexcept { return std::tie(easting_m, northing_m, alt_m, zone, north); }
        auto members() const noexcept { return std::tie(easting_m, northing_m, alt_m, zone, north); }
    };

    inline std::ostream &operator<<(std::ostream &os, const UTM &u) {
        return os << "UTM{zone=" << u.zone << (u.north ? "N" : "S") << ", e=" << u.easting_m << "m, n=" << u.northing_m
                  << "m, alt=" << u.alt_m << "m}";
    }

} // namespace concord::earth
