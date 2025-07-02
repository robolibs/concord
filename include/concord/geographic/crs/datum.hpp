#pragma once

namespace concord {

    struct Datum {
        double lat = 0.0;
        double lon = 0.0;
        double alt = 0.0;

        inline bool is_set() const { return lat != 0.0 && lon != 0.0; }

        inline bool operator==(const Datum& other) const {
            return lat == other.lat && lon == other.lon && alt == other.alt;
        }

        inline bool operator!=(const Datum& other) const {
            return !(*this == other);
        }
    };

} // namespace concord
