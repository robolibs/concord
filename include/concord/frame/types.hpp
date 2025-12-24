#pragma once

#include <datapod/datapod.hpp>
#include <ostream>
#include <tuple>

#include "tags.hpp"

namespace concord::frame {

    // ============================================================================
    // ENU - East-North-Up local tangent frame
    // ============================================================================
    struct ENU {
        static constexpr const char *name = "enu";
        static constexpr FrameTag tag = FrameTag::LocalTangent;

        dp::Point p{};

        // Constructors
        ENU() = default;
        explicit ENU(const dp::Point &pt) : p(pt) {}
        ENU(double east, double north, double up) : p{east, north, up} {}

        // Semantic accessors
        double &east() { return p.x; }
        double &north() { return p.y; }
        double &up() { return p.z; }
        double east() const { return p.x; }
        double north() const { return p.y; }
        double up() const { return p.z; }

        // Generic accessors
        double &x() { return p.x; }
        double &y() { return p.y; }
        double &z() { return p.z; }
        double x() const { return p.x; }
        double y() const { return p.y; }
        double z() const { return p.z; }

        bool is_set() const noexcept { return p.is_set(); }

        auto members() noexcept { return std::tie(p); }
        auto members() const noexcept { return std::tie(p); }
    };

    template <> struct FrameTraits<ENU> {
        static constexpr const char *name = "enu";
        static constexpr FrameTag tag = FrameTag::LocalTangent;
    };

    inline std::ostream &operator<<(std::ostream &os, const ENU &v) {
        return os << "ENU{e=" << v.east() << ", n=" << v.north() << ", u=" << v.up() << "}";
    }

    // ============================================================================
    // NED - North-East-Down local tangent frame
    // ============================================================================
    struct NED {
        static constexpr const char *name = "ned";
        static constexpr FrameTag tag = FrameTag::LocalTangent;

        dp::Point p{};

        NED() = default;
        explicit NED(const dp::Point &pt) : p(pt) {}
        NED(double north, double east, double down) : p{north, east, down} {}

        // Semantic accessors
        double &north() { return p.x; }
        double &east() { return p.y; }
        double &down() { return p.z; }
        double north() const { return p.x; }
        double east() const { return p.y; }
        double down() const { return p.z; }

        double &x() { return p.x; }
        double &y() { return p.y; }
        double &z() { return p.z; }
        double x() const { return p.x; }
        double y() const { return p.y; }
        double z() const { return p.z; }

        bool is_set() const noexcept { return p.is_set(); }

        auto members() noexcept { return std::tie(p); }
        auto members() const noexcept { return std::tie(p); }
    };

    template <> struct FrameTraits<NED> {
        static constexpr const char *name = "ned";
        static constexpr FrameTag tag = FrameTag::LocalTangent;
    };

    inline std::ostream &operator<<(std::ostream &os, const NED &v) {
        return os << "NED{n=" << v.north() << ", e=" << v.east() << ", d=" << v.down() << "}";
    }

    // ============================================================================
    // FRD - Forward-Right-Down body frame (aerospace convention)
    // ============================================================================
    struct FRD {
        static constexpr const char *name = "frd";
        static constexpr FrameTag tag = FrameTag::Body;

        dp::Point p{};

        FRD() = default;
        explicit FRD(const dp::Point &pt) : p(pt) {}
        FRD(double forward, double right, double down) : p{forward, right, down} {}

        // Semantic accessors
        double &forward() { return p.x; }
        double &right() { return p.y; }
        double &down() { return p.z; }
        double forward() const { return p.x; }
        double right() const { return p.y; }
        double down() const { return p.z; }

        double &x() { return p.x; }
        double &y() { return p.y; }
        double &z() { return p.z; }
        double x() const { return p.x; }
        double y() const { return p.y; }
        double z() const { return p.z; }

        bool is_set() const noexcept { return p.is_set(); }

        auto members() noexcept { return std::tie(p); }
        auto members() const noexcept { return std::tie(p); }
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
    // ============================================================================
    struct FLU {
        static constexpr const char *name = "flu";
        static constexpr FrameTag tag = FrameTag::Body;

        dp::Point p{};

        FLU() = default;
        explicit FLU(const dp::Point &pt) : p(pt) {}
        FLU(double forward, double left, double up) : p{forward, left, up} {}

        // Semantic accessors
        double &forward() { return p.x; }
        double &left() { return p.y; }
        double &up() { return p.z; }
        double forward() const { return p.x; }
        double left() const { return p.y; }
        double up() const { return p.z; }

        double &x() { return p.x; }
        double &y() { return p.y; }
        double &z() { return p.z; }
        double x() const { return p.x; }
        double y() const { return p.y; }
        double z() const { return p.z; }

        bool is_set() const noexcept { return p.is_set(); }

        auto members() noexcept { return std::tie(p); }
        auto members() const noexcept { return std::tie(p); }
    };

    template <> struct FrameTraits<FLU> {
        static constexpr const char *name = "flu";
        static constexpr FrameTag tag = FrameTag::Body;
    };

    inline std::ostream &operator<<(std::ostream &os, const FLU &v) {
        return os << "FLU{f=" << v.forward() << ", l=" << v.left() << ", u=" << v.up() << "}";
    }

} // namespace concord::frame
