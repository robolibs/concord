#pragma once

#include <type_traits>

namespace concord::frame {

    // Frame categories
    enum class FrameTag {
        Geocentric,   // WGS, ECF - Earth-centered frames
        LocalTangent, // ENU, NED - Local tangent plane frames
        Body,         // FRD, FLU - Vehicle body frames
        Sensor        // Camera, IMU - Sensor-mounted frames
    };

    // Frame traits template - specialize for each frame type
    template <typename Frame> struct FrameTraits {
        static_assert(sizeof(Frame) == 0, "FrameTraits not specialized for this frame type");
    };

    // Compile-time validator: ensures frames match
    template <typename Frame1, typename Frame2> struct FrameValidator {
        static constexpr void validate() {
            static_assert(std::is_same_v<Frame1, Frame2>, "Operation requires matching frame types");
        }
    };

    // Compile-time validator: ensures frames have specific tag
    template <FrameTag Required, typename... Frames> struct FrameTagValidator {
        static constexpr void validate() {
            static_assert(((FrameTraits<Frames>::tag == Required) && ...), "Operation requires specific frame tag");
        }
    };

    // Compile-time validator: ensures frames have same tag
    template <typename Frame1, typename Frame2> struct SameTagValidator {
        static constexpr void validate() {
            static_assert(FrameTraits<Frame1>::tag == FrameTraits<Frame2>::tag,
                          "Frames must have the same tag for this operation");
        }
    };

    // Helper to check if a type has frame traits
    template <typename T, typename = void> struct has_frame_traits : std::false_type {};

    template <typename T> struct has_frame_traits<T, std::void_t<decltype(FrameTraits<T>::tag)>> : std::true_type {};

    template <typename T> inline constexpr bool has_frame_traits_v = has_frame_traits<T>::value;

} // namespace concord::frame
