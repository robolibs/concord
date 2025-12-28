#pragma once

#include <datapod/datapod.hpp>
#include <ostream>
#include <tuple>

#include "wgs84.hpp"

namespace concord::earth {

    // Unit for angular values
    enum class AngleUnit { Deg, Rad };

    // ============================================================================
    // WGS - WGS84 geodetic coordinates (extends dp::Geo)
    //
    // Inherits from dp::Geo for data storage, adds concord-specific methods:
    // - Unit conversion (degrees/radians)
    // - Factory from radians
    // - Compatibility with concord conversion functions
    //
    // Data fields (inherited from dp::Geo):
    // - latitude: degrees, positive = north
    // - longitude: degrees, positive = east
    // - altitude: meters above WGS84 ellipsoid
    // ============================================================================
    struct WGS : dp::Geo {
        // Inherit constructors from dp::Geo
        using dp::Geo::Geo;

        // Default constructor
        WGS() = default;

        // Constructor matching old API (lat, lon, alt in degrees)
        WGS(double lat_deg, double lon_deg, double alt_m = 0.0) : dp::Geo{lat_deg, lon_deg, alt_m} {}

        // Construct from dp::Geo
        WGS(const dp::Geo &geo) : dp::Geo{geo.latitude, geo.longitude, geo.altitude} {}

        // Factory from radians
        static WGS from_radians(double lat_rad, double lon_rad, double alt_m = 0.0) {
            return WGS{lat_rad * wgs84::rad_to_deg, lon_rad * wgs84::rad_to_deg, alt_m};
        }

        // Accessors with unit conversion (concord-specific)
        double lat_rad() const { return latitude * wgs84::deg_to_rad; }
        double lon_rad() const { return longitude * wgs84::deg_to_rad; }

        // Convert to dp::Geo (implicit via inheritance, but explicit method for clarity)
        const dp::Geo &geo() const { return *this; }
        dp::Geo &geo() { return *this; }
    };

    inline std::ostream &operator<<(std::ostream &os, const WGS &w) {
        return os << "WGS{lat=" << w.latitude << "deg, lon=" << w.longitude << "deg, alt=" << w.altitude << "m}";
    }

    // ============================================================================
    // ECF - Earth-Centered Fixed (ECEF) coordinates
    //
    // Uses dp::Point for storage. ECEF doesn't have a direct dp:: equivalent,
    // so we wrap dp::Point with semantic accessors.
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

        // Access underlying point
        const dp::Point &point() const { return p; }
        dp::Point &point() { return p; }
    };

    inline std::ostream &operator<<(std::ostream &os, const ECF &e) {
        return os << "ECF{x=" << e.p.x << "m, y=" << e.p.y << "m, z=" << e.p.z << "m}";
    }

    // ============================================================================
    // UTM - Universal Transverse Mercator coordinates (extends dp::Utm)
    //
    // Inherits from dp::Utm for data storage, adds concord-specific methods.
    //
    // Data fields (inherited from dp::Utm):
    // - zone: UTM zone number [1-60]
    // - band: UTM latitude band letter [C-X]
    // - easting: meters from zone central meridian
    // - northing: meters from equator
    // - altitude: meters above WGS84 ellipsoid
    // ============================================================================
    struct UTM : dp::Utm {
        // Inherit constructors from dp::Utm
        using dp::Utm::Utm;

        // Default constructor
        UTM() = default;

        // Constructor matching old API (easting, northing, alt, zone, north)
        UTM(double easting_val, double northing_val, double alt, int z, bool n)
            : dp::Utm{z, n ? 'N' : 'S', easting_val, northing_val, alt} {}

        // Construct from dp::Utm
        UTM(const dp::Utm &u) : dp::Utm{u.zone, u.band, u.easting, u.northing, u.altitude} {}

        // North/south hemisphere check
        bool is_north() const { return is_northern(); }

        // Convert to dp::Utm
        const dp::Utm &utm() const { return *this; }
        dp::Utm &utm() { return *this; }
    };

    inline std::ostream &operator<<(std::ostream &os, const UTM &u) {
        return os << "UTM{zone=" << u.zone << u.band << ", e=" << u.easting << "m, n=" << u.northing
                  << "m, alt=" << u.altitude << "m}";
    }

} // namespace concord::earth
