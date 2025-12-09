#include "concord/geographic/crs/wgs.hpp"
#include "concord/geographic/crs/enu.hpp"
#include <cmath>

namespace concord {

    WGS::WGS(double lat_, double lon_, double alt_) : lat(lat_), lon(lon_), alt(alt_) {
        validation::validate_latitude(lat);
        validation::validate_longitude(lon);
        validation::validate_altitude(alt);
    }

    WGS::operator Datum() const noexcept { 
        return Datum{lat, lon, alt}; 
    }

    bool WGS::is_set() const { 
        return lat != 0.0 && lon != 0.0; 
    }

    WGS WGS::operator+(const WGS &offset) const {
        return WGS{lat + offset.lat, lon + offset.lon, alt + offset.alt};
    }

    WGS WGS::operator-(const WGS &offset) const {
        return WGS{lat - offset.lat, lon - offset.lon, alt - offset.alt};
    }

    double WGS::distance_to(const WGS &other) const {
        validation::validate_finite(lat, "latitude");
        validation::validate_finite(lon, "longitude");
        validation::validate_finite(other.lat, "other latitude");
        validation::validate_finite(other.lon, "other longitude");

        const double R = 6371000.0; // Earth radius in meters
        double dlat = (other.lat - lat) * M_PI / 180.0;
        double dlon = (other.lon - lon) * M_PI / 180.0;
        double a = std::sin(dlat / 2) * std::sin(dlat / 2) + 
                   std::cos(lat * M_PI / 180.0) * std::cos(other.lat * M_PI / 180.0) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);
        double c = 2 * safe_math::safe_asin(safe_math::safe_sqrt(a, "haversine"), "haversine");
        return R * c;
    }

    double WGS::bearing_to(const WGS &other) const {
        double dlon = (other.lon - lon) * M_PI / 180.0;
        double lat1_rad = lat * M_PI / 180.0;
        double lat2_rad = other.lat * M_PI / 180.0;

        double y = std::sin(dlon) * std::cos(lat2_rad);
        double x = std::cos(lat1_rad) * std::sin(lat2_rad) - 
                   std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon);
        double bearing = std::atan2(y, x) * 180.0 / M_PI;
        return std::fmod(bearing + 360.0, 360.0);
    }

    bool WGS::operator==(const WGS &other) const {
        return lat == other.lat && lon == other.lon && alt == other.alt;
    }

    bool WGS::operator!=(const WGS &other) const { 
        return !(*this == other); 
    }

} // namespace concord
