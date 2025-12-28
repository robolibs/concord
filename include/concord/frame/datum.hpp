#pragma once

#include <datapod/datapod.hpp>

#include "../earth/types.hpp"
#include <tuple>

namespace concord::frame {

    /**
     * @brief Datum - Reference point for local tangent plane frames
     *
     * Defines the origin for ENU/NED local coordinate systems.
     * Uses dp::Geo (via earth::WGS) for storage.
     */
    struct Datum {
        earth::WGS origin{};

        Datum() = default;
        explicit Datum(const earth::WGS &wgs) : origin(wgs) {}
        explicit Datum(const dp::Geo &geo) : origin(geo) {}
        Datum(double lat_deg, double lon_deg, double alt_m = 0.0) : origin{lat_deg, lon_deg, alt_m} {}

        auto members() noexcept { return std::tie(origin); }
        auto members() const noexcept { return std::tie(origin); }

        bool is_set() const noexcept { return origin.is_set(); }

        // Access origin as dp::Geo
        const dp::Geo &geo() const { return origin.geo(); }
        dp::Geo &geo() { return origin.geo(); }
    };

} // namespace concord::frame
