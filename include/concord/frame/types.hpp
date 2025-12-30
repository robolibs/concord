#pragma once

#include <datapod/datapod.hpp>
#include <ostream>
#include <tuple>

#include "tags.hpp"

namespace concord::frame {

    // ============================================================================
    // ENU - East-North-Up local tangent frame
    //
    // Extends dp::Loc to carry both local coordinates AND the WGS84 origin.
    // This makes ENU self-contained - it always knows its global reference.
    //
    // Convention: local.x = East, local.y = North, local.z = Up
    // ============================================================================
    struct ENU : dp::Loc {
        static constexpr const char *name = "enu";
        static constexpr FrameTag tag = FrameTag::LocalTangent;

        // Inherit constructors from dp::Loc
        using dp::Loc::Loc;

        // Default constructor
        ENU() = default;

        // Construct from dp::Loc directly
        ENU(const dp::Loc &loc) : dp::Loc{loc} {}

        // Construct with local coords and origin
        ENU(double east, double north, double up, const dp::Geo &ref) : dp::Loc{dp::Point{east, north, up}, ref} {}

        // Construct with dp::Point and origin
        ENU(const dp::Point &pt, const dp::Geo &ref) : dp::Loc{pt, ref} {}

        // Semantic accessors (ENU convention: x=East, y=North, z=Up)
        double &east() { return local.x; }
        double &north() { return local.y; }
        double &up() { return local.z; }
        double east() const { return local.x; }
        double north() const { return local.y; }
        double up() const { return local.z; }

        // Generic accessors
        double &x() { return local.x; }
        double &y() { return local.y; }
        double &z() { return local.z; }
        double x() const { return local.x; }
        double y() const { return local.y; }
        double z() const { return local.z; }

        // Access underlying point (for compatibility)
        const dp::Point &point() const { return local; }
        dp::Point &point() { return local; }

        // Reference origin access (inherited from dp::Loc as 'origin')
        const dp::Geo &ref() const { return origin; }
        dp::Geo &ref() { return origin; }
    };

    template <> struct FrameTraits<ENU> {
        static constexpr const char *name = "enu";
        static constexpr FrameTag tag = FrameTag::LocalTangent;
    };

    inline std::ostream &operator<<(std::ostream &os, const ENU &v) {
        return os << "ENU{e=" << v.east() << ", n=" << v.north() << ", u=" << v.up() << " @(" << v.origin.latitude
                  << "," << v.origin.longitude << ")}";
    }

    // ============================================================================
    // NED - North-East-Down local tangent frame
    //
    // Extends dp::Loc to carry both local coordinates AND the WGS84 origin.
    // Internally stores in NED convention: local.x = North, local.y = East, local.z = Down
    //
    // Convention: local.x = North, local.y = East, local.z = Down
    // ============================================================================
    struct NED : dp::Loc {
        static constexpr const char *name = "ned";
        static constexpr FrameTag tag = FrameTag::LocalTangent;

        // Inherit constructors from dp::Loc
        using dp::Loc::Loc;

        // Default constructor
        NED() = default;

        // Construct from dp::Loc (assumes dp::Loc is in ENU, converts to NED)
        // ENU: x=East, y=North, z=Up -> NED: x=North, y=East, z=Down
        explicit NED(const dp::Loc &loc) : dp::Loc{dp::Point{loc.local.y, loc.local.x, -loc.local.z}, loc.origin} {}

        // Construct with local coords (NED convention) and origin
        NED(double north, double east, double down, const dp::Geo &ref) : dp::Loc{dp::Point{north, east, down}, ref} {}

        // Construct with dp::Point (NED convention) and origin
        NED(const dp::Point &pt, const dp::Geo &ref) : dp::Loc{pt, ref} {}

        // Semantic accessors (NED convention: x=North, y=East, z=Down)
        double &north() { return local.x; }
        double &east() { return local.y; }
        double &down() { return local.z; }
        double north() const { return local.x; }
        double east() const { return local.y; }
        double down() const { return local.z; }

        // Generic accessors
        double &x() { return local.x; }
        double &y() { return local.y; }
        double &z() { return local.z; }
        double x() const { return local.x; }
        double y() const { return local.y; }
        double z() const { return local.z; }

        // Access underlying point (for compatibility)
        const dp::Point &point() const { return local; }
        dp::Point &point() { return local; }

        // Reference origin access (inherited from dp::Loc as 'origin')
        const dp::Geo &ref() const { return origin; }
        dp::Geo &ref() { return origin; }
    };

    template <> struct FrameTraits<NED> {
        static constexpr const char *name = "ned";
        static constexpr FrameTag tag = FrameTag::LocalTangent;
    };

    inline std::ostream &operator<<(std::ostream &os, const NED &v) {
        return os << "NED{n=" << v.north() << ", e=" << v.east() << ", d=" << v.down() << " @(" << v.origin.latitude
                  << "," << v.origin.longitude << ")}";
    }

    // ============================================================================
    // FRD - Forward-Right-Down body frame (aerospace convention)
    //
    // Extends dp::Point for body-relative coordinates.
    // Body frames don't have a global origin - they're relative to the robot body.
    //
    // Convention: x = Forward, y = Right, z = Down
    // ============================================================================
    struct FRD : dp::Point {
        static constexpr const char *name = "frd";
        static constexpr FrameTag tag = FrameTag::Body;

        // Inherit constructors from dp::Point
        using dp::Point::Point;

        FRD() = default;
        FRD(const dp::Point &pt) : dp::Point{pt} {}
        FRD(double forward, double right, double down) : dp::Point{forward, right, down} {}

        // Semantic accessors (FRD convention: x=Forward, y=Right, z=Down)
        double &forward() { return x; }
        double &right() { return y; }
        double &down() { return z; }
        double forward() const { return x; }
        double right() const { return y; }
        double down() const { return z; }

        // Access as dp::Point (for compatibility)
        const dp::Point &point() const { return *this; }
        dp::Point &point() { return *this; }
    };

    template <> struct FrameTraits<FRD> {
        static constexpr const char *name = "frd";
        static constexpr FrameTag tag = FrameTag::Body;
    };

    inline std::ostream &operator<<(std::ostream &os, const FRD &v) {
        return os << "FRD{f=" << v.forward() << ", r=" << v.right() << ", d=" << v.down() << "}";
    }

    // ============================================================================
    // FLU - Forward-Left-Up body frame (ROS convention)
    //
    // Extends dp::Point for body-relative coordinates.
    // Body frames don't have a global origin - they're relative to the robot body.
    //
    // Convention: x = Forward, y = Left, z = Up
    // ============================================================================
    struct FLU : dp::Point {
        static constexpr const char *name = "flu";
        static constexpr FrameTag tag = FrameTag::Body;

        // Inherit constructors from dp::Point
        using dp::Point::Point;

        FLU() = default;
        FLU(const dp::Point &pt) : dp::Point{pt} {}
        FLU(double forward, double left, double up) : dp::Point{forward, left, up} {}

        // Semantic accessors (FLU convention: x=Forward, y=Left, z=Up)
        double &forward() { return x; }
        double &left() { return y; }
        double &up() { return z; }
        double forward() const { return x; }
        double left() const { return y; }
        double up() const { return z; }

        // Access as dp::Point (for compatibility)
        const dp::Point &point() const { return *this; }
        dp::Point &point() { return *this; }
    };

    template <> struct FrameTraits<FLU> {
        static constexpr const char *name = "flu";
        static constexpr FrameTag tag = FrameTag::Body;
    };

    inline std::ostream &operator<<(std::ostream &os, const FLU &v) {
        return os << "FLU{f=" << v.forward() << ", l=" << v.left() << ", u=" << v.up() << "}";
    }

} // namespace concord::frame
