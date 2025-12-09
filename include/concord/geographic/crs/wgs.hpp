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

        WGS(double lat_, double lon_, double alt_);
        WGS() = default;
        
        operator Datum() const noexcept;
        bool is_set() const;

        ENU toENU(const Datum &datum) const;

        // Mathematical operations (for small displacements)
        WGS operator+(const WGS &offset) const;
        WGS operator-(const WGS &offset) const;

        // Distance calculation using haversine formula
        double distance_to(const WGS &other) const;

        // Bearing calculation
        double bearing_to(const WGS &other) const;

        bool operator==(const WGS &other) const;
        bool operator!=(const WGS &other) const;
    };

} // namespace concord
