#pragma once

#include "../wgs_to_utm.hpp"
#include "wgs.hpp"
#include <string>

namespace concord {

    // UTM coordinate type
    struct UTM {
        double easting = 0.0;
        double northing = 0.0;
        double altitude = 0.0;
        int zone = 0;
        bool is_north = true;

        UTM() = default;
        UTM(double e, double n, double alt, int z, bool north)
            : easting(e), northing(n), altitude(alt), zone(z), is_north(north) {}

        WGS toWGS() const {
            auto [lat, lon] = utm_to_wgs(easting, northing, zone, is_north);
            return WGS{lat, lon, altitude};
        }

        static UTM fromWGS(const WGS &wgs) {
            auto [e, n, z, north] = wgs_to_utm(wgs.lat, wgs.lon);
            return UTM{e, n, wgs.alt, z, north};
        }

        bool is_set() const { return easting != 0.0 || northing != 0.0; }

        std::string toString() const {
            return std::to_string(zone) + (is_north ? "N " : "S ") + std::to_string(easting) + " " +
                   std::to_string(northing);
        }

        inline bool operator==(const UTM& other) const {
            return easting == other.easting && northing == other.northing && 
                   altitude == other.altitude && zone == other.zone && is_north == other.is_north;
        }

        inline bool operator!=(const UTM& other) const {
            return !(*this == other);
        }
    };

} // namespace concord
