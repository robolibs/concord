#pragma once

#include <datapod/datapod.hpp>
#include <tuple>

namespace concord::earth {

    struct WGS {
        double lat_deg = 0.0;
        double lon_deg = 0.0;
        double alt_m = 0.0;

        auto members() noexcept { return std::tie(lat_deg, lon_deg, alt_m); }
        auto members() const noexcept { return std::tie(lat_deg, lon_deg, alt_m); }

        bool is_set() const noexcept { return lat_deg != 0.0 || lon_deg != 0.0 || alt_m != 0.0; }
    };

    struct ECF {
        dp::Point p; // x,y,z in meters

        auto members() noexcept { return std::tie(p); }
        auto members() const noexcept { return std::tie(p); }

        bool is_set() const noexcept { return p.is_set(); }
    };

    struct UTM {
        double easting_m = 0.0;
        double northing_m = 0.0;
        double alt_m = 0.0;
        int zone = 0;
        bool north = true;

        auto members() noexcept { return std::tie(easting_m, northing_m, alt_m, zone, north); }
        auto members() const noexcept { return std::tie(easting_m, northing_m, alt_m, zone, north); }

        bool is_set() const noexcept { return easting_m != 0.0 || northing_m != 0.0; }
    };

} // namespace concord::earth
