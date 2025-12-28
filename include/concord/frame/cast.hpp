#pragma once

#include "tags.hpp"
#include "types.hpp"

namespace concord::frame {

    // Primary template - will fail if no specialization exists
    template <typename To, typename From> To frame_cast(const From &from) {
        static_assert(sizeof(To) == 0, "No frame_cast specialization for this conversion");
        return To{};
    }

    // ============================================================================
    // ENU <-> NED conversions (preserves origin)
    // ============================================================================
    template <> inline NED frame_cast<NED, ENU>(const ENU &enu) {
        // ENU(e,n,u) -> NED(n,e,-u), preserving origin
        return NED{enu.north(), enu.east(), -enu.up(), enu.origin};
    }

    template <> inline ENU frame_cast<ENU, NED>(const NED &ned) {
        // NED(n,e,d) -> ENU(e,n,-d), preserving origin
        return ENU{ned.east(), ned.north(), -ned.down(), ned.origin};
    }

    // ============================================================================
    // FRD <-> FLU conversions (body frames, no origin)
    // ============================================================================
    template <> inline FLU frame_cast<FLU, FRD>(const FRD &frd) {
        // FRD(f,r,d) -> FLU(f,-r,-d)
        return FLU{frd.forward(), -frd.right(), -frd.down()};
    }

    template <> inline FRD frame_cast<FRD, FLU>(const FLU &flu) {
        // FLU(f,l,u) -> FRD(f,-l,-u)
        return FRD{flu.forward(), -flu.left(), -flu.up()};
    }

    // ============================================================================
    // Identity conversions (same type)
    // ============================================================================
    template <> inline ENU frame_cast<ENU, ENU>(const ENU &v) { return v; }

    template <> inline NED frame_cast<NED, NED>(const NED &v) { return v; }

    template <> inline FRD frame_cast<FRD, FRD>(const FRD &v) { return v; }

    template <> inline FLU frame_cast<FLU, FLU>(const FLU &v) { return v; }

} // namespace concord::frame
