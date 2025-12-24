#pragma once

#include <datapod/datapod.hpp>

#include "../earth/types.hpp"
#include <tuple>

namespace concord::frame {

    struct Datum {
        earth::WGS origin{};

        Datum() = default;
        explicit Datum(const earth::WGS &wgs) : origin(wgs) {}
        Datum(double lat_deg, double lon_deg, double alt_m) : origin{lat_deg, lon_deg, alt_m} {}

        auto members() noexcept { return std::tie(origin); }
        auto members() const noexcept { return std::tie(origin); }

        bool is_set() const noexcept { return origin.is_set(); }
    };

} // namespace concord::frame
