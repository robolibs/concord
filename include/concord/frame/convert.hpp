#pragma once

#include <datapod/datapod.hpp>

#include "../earth/local_axes.hpp"
#include "../earth/types.hpp"
#include "../earth/utm.hpp"
#include "../earth/wgs84.hpp"
#include "../earth/wgs_ecf.hpp"
#include "datum.hpp"
#include "types.hpp"
#include <type_traits>
#include <utility>

namespace concord::frame {

    // ============================================================================
    // WGS <-> ENU conversions
    //
    // ENU now carries its origin (dp::Geo), so conversions are self-contained.
    // ============================================================================

    /**
     * @brief Convert WGS84 coordinates to ENU relative to a reference point
     * @param ref The reference origin (dp::Geo or Datum)
     * @param wgs The WGS84 point to convert
     * @return ENU coordinates with the reference origin embedded
     */
    inline ENU to_enu(const dp::Geo &ref, const earth::WGS &wgs) {
        const earth::WGS origin{ref};
        const earth::ECF p_ecf = earth::to_ecf(wgs);
        const earth::ECF o_ecf = earth::to_ecf(origin);

        // ECF extends dp::Point, so x/y/z are direct members
        const dp::Point d{p_ecf.x - o_ecf.x, p_ecf.y - o_ecf.y, p_ecf.z - o_ecf.z};

        const double lat_rad = origin.lat_rad();
        const double lon_rad = origin.lon_rad();
        const auto R = earth::R_enu_from_ecf(lat_rad, lon_rad);

        return ENU{earth::mat_mul(R, d), ref};
    }

    /**
     * @brief Convert ENU coordinates back to WGS84
     *
     * ENU carries its origin, so no external reference needed!
     * @param enu The ENU coordinates (with embedded origin)
     * @return WGS84 coordinates
     */
    inline earth::WGS to_wgs(const ENU &enu) {
        const earth::WGS origin{enu.origin};
        const double lat_rad = origin.lat_rad();
        const double lon_rad = origin.lon_rad();

        const auto R = earth::R_enu_from_ecf(lat_rad, lon_rad);
        const auto Rt = earth::transpose(R);
        const dp::Point d = earth::mat_mul(Rt, enu.local);

        const earth::ECF o_ecf = earth::to_ecf(origin);
        // ECF extends dp::Point, so x/y/z are direct members
        const earth::ECF p_ecf{o_ecf.x + d.x, o_ecf.y + d.y, o_ecf.z + d.z};
        return earth::to_wgs(p_ecf);
    }

    // ============================================================================
    // WGS <-> NED conversions
    //
    // NED now carries its origin (dp::Geo), so conversions are self-contained.
    // ============================================================================

    /**
     * @brief Convert WGS84 coordinates to NED relative to a reference point
     * @param ref The reference origin (dp::Geo or Datum)
     * @param wgs The WGS84 point to convert
     * @return NED coordinates with the reference origin embedded
     */
    inline NED to_ned(const dp::Geo &ref, const earth::WGS &wgs) {
        const ENU enu = to_enu(ref, wgs);
        // ENU to NED: swap x/y, negate z
        return NED{enu.north(), enu.east(), -enu.up(), ref};
    }

    /**
     * @brief Convert NED coordinates back to WGS84
     *
     * NED carries its origin, so no external reference needed!
     * @param ned The NED coordinates (with embedded origin)
     * @return WGS84 coordinates
     */
    inline earth::WGS to_wgs(const NED &ned) {
        // NED to ENU: swap x/y, negate z
        const ENU enu{ned.east(), ned.north(), -ned.down(), ned.origin};
        return to_wgs(enu);
    }

    // ============================================================================
    // ENU <-> NED conversions (same origin)
    // ============================================================================

    /**
     * @brief Convert ENU to NED (preserves origin)
     */
    inline NED to_ned(const ENU &enu) { return NED{enu.north(), enu.east(), -enu.up(), enu.origin}; }

    /**
     * @brief Convert NED to ENU (preserves origin)
     */
    inline ENU to_enu(const NED &ned) { return ENU{ned.east(), ned.north(), -ned.down(), ned.origin}; }

} // namespace concord::frame

namespace concord {

    namespace detail {

        template <typename Target, typename Source>
        dp::Result<Target> convert_one(const Source &value, const dp::Optional<dp::Geo> &ref) {
            if constexpr (std::is_same_v<Source, earth::WGS> && std::is_same_v<Target, earth::ECF>) {
                return dp::Result<Target>::ok(earth::to_ecf(value));
            } else if constexpr (std::is_same_v<Source, earth::ECF> && std::is_same_v<Target, earth::WGS>) {
                return dp::Result<Target>::ok(earth::to_wgs(value));
            } else if constexpr (std::is_same_v<Source, earth::WGS> && std::is_same_v<Target, earth::UTM>) {
                return earth::to_utm(value);
            } else if constexpr (std::is_same_v<Source, earth::UTM> && std::is_same_v<Target, earth::WGS>) {
                return earth::to_wgs(value);
            } else if constexpr (std::is_same_v<Source, earth::WGS> && std::is_same_v<Target, frame::ENU>) {
                if (!ref) {
                    return dp::Result<Target>::err(dp::Error::invalid_argument("WGS->ENU requires reference origin"));
                }
                return dp::Result<Target>::ok(frame::to_enu(*ref, value));
            } else if constexpr (std::is_same_v<Source, frame::ENU> && std::is_same_v<Target, earth::WGS>) {
                // ENU now carries its origin - no external ref needed!
                return dp::Result<Target>::ok(frame::to_wgs(value));
            } else if constexpr (std::is_same_v<Source, earth::WGS> && std::is_same_v<Target, frame::NED>) {
                if (!ref) {
                    return dp::Result<Target>::err(dp::Error::invalid_argument("WGS->NED requires reference origin"));
                }
                return dp::Result<Target>::ok(frame::to_ned(*ref, value));
            } else if constexpr (std::is_same_v<Source, frame::NED> && std::is_same_v<Target, earth::WGS>) {
                // NED now carries its origin - no external ref needed!
                return dp::Result<Target>::ok(frame::to_wgs(value));
            } else if constexpr (std::is_same_v<Source, frame::ENU> && std::is_same_v<Target, frame::NED>) {
                return dp::Result<Target>::ok(frame::to_ned(value));
            } else if constexpr (std::is_same_v<Source, frame::NED> && std::is_same_v<Target, frame::ENU>) {
                return dp::Result<Target>::ok(frame::to_enu(value));
            } else {
                static_assert(sizeof(Target) == 0, "Unsupported conversion");
            }
        }

    } // namespace detail

    /**
     * @brief Fluent builder for coordinate conversions
     *
     * Usage:
     *   // WGS -> ENU (requires reference)
     *   auto enu = convert(wgs).withRef({52.0, 13.0, 0.0}).to<ENU>().build();
     *
     *   // ENU -> WGS (no reference needed - ENU carries it!)
     *   auto wgs = convert(enu).to<WGS>().build();
     *
     *   // Chained conversions
     *   auto utm = convert(wgs).to<ECF>().to<WGS>().to<UTM>().build();
     */
    template <typename T> class ConvertBuilder {
      public:
        explicit ConvertBuilder(T value) : state_(dp::Result<T>::ok(std::move(value))) {}
        explicit ConvertBuilder(dp::Result<T> state) : state_(std::move(state)) {}

        /**
         * @brief Set reference origin for WGS -> ENU/NED conversions
         */
        ConvertBuilder &withRef(const dp::Geo &ref) {
            ref_ = ref;
            return *this;
        }

        // Alias for backward compatibility
        ConvertBuilder &withDatum(const dp::Geo &ref) { return withRef(ref); }

        template <typename Target> ConvertBuilder<Target> to() const {
            if (state_.is_err()) {
                return ConvertBuilder<Target>(dp::Result<Target>::err(state_.error()), ref_);
            }
            return ConvertBuilder<Target>(detail::convert_one<Target>(state_.value(), ref_), ref_);
        }

        dp::Result<T> build() const { return state_; }

      private:
        template <typename> friend class ConvertBuilder;

        ConvertBuilder(dp::Result<T> state, dp::Optional<dp::Geo> ref)
            : state_(std::move(state)), ref_(std::move(ref)) {}

        dp::Result<T> state_;
        dp::Optional<dp::Geo> ref_{};
    };

    template <typename T> ConvertBuilder<std::decay_t<T>> convert(T &&value) {
        return ConvertBuilder<std::decay_t<T>>(std::forward<T>(value));
    }

} // namespace concord
