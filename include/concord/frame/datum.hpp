#pragma once

#include <datapod/datapod.hpp>

#include "../earth/types.hpp"

namespace concord::frame {

    /**
     * @brief Datum/Ref - Reference point for local tangent plane frames
     *
     * Alias to dp::Geo (WGS84 geodetic coordinates).
     * Defines the origin for ENU/NED local coordinate systems.
     *
     * Now that ENU/NED extend dp::Loc, they carry their origin internally.
     * These aliases are provided for semantic clarity when specifying origins.
     */
    using Datum = dp::Geo;
    using Ref = dp::Geo;

} // namespace concord::frame

namespace concord {

    // Top-level aliases for convenience
    using Datum = dp::Geo;
    using Ref = dp::Geo;

} // namespace concord
