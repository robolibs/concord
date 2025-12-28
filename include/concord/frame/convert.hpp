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

    inline ENU to_enu(const Datum &datum, const earth::WGS &wgs) {
        const earth::ECF p_ecf = earth::to_ecf(wgs);
        const earth::ECF o_ecf = earth::to_ecf(datum.origin);

        const dp::Point d{p_ecf.p.x - o_ecf.p.x, p_ecf.p.y - o_ecf.p.y, p_ecf.p.z - o_ecf.p.z};

        const double lat_rad = datum.origin.lat_rad();
        const double lon_rad = datum.origin.lon_rad();
        const auto R = earth::R_enu_from_ecf(lat_rad, lon_rad);

        return ENU{earth::mat_mul(R, d)};
    }

    inline earth::WGS to_wgs(const Datum &datum, const ENU &enu) {
        const double lat_rad = datum.origin.lat_rad();
        const double lon_rad = datum.origin.lon_rad();

        const auto R = earth::R_enu_from_ecf(lat_rad, lon_rad);
        const auto Rt = earth::transpose(R);
        const dp::Point d = earth::mat_mul(Rt, enu.p);

        const earth::ECF o_ecf = earth::to_ecf(datum.origin);
        const earth::ECF p_ecf{dp::Point{o_ecf.p.x + d.x, o_ecf.p.y + d.y, o_ecf.p.z + d.z}};
        return earth::to_wgs(p_ecf);
    }

    inline NED to_ned(const Datum &datum, const earth::WGS &wgs) {
        const ENU enu = to_enu(datum, wgs);
        return NED{dp::Point{enu.p.y, enu.p.x, -enu.p.z}};
    }

    inline earth::WGS to_wgs(const Datum &datum, const NED &ned) {
        const ENU enu{dp::Point{ned.p.y, ned.p.x, -ned.p.z}};
        return to_wgs(datum, enu);
    }

    inline NED to_ned(const ENU &enu) { return NED{dp::Point{enu.p.y, enu.p.x, -enu.p.z}}; }
    inline ENU to_enu(const NED &ned) { return ENU{dp::Point{ned.p.y, ned.p.x, -ned.p.z}}; }

} // namespace concord::frame

namespace concord {

    namespace detail {

        template <typename Target, typename Source>
        dp::Result<Target> convert_one(const Source &value, const dp::Optional<frame::Datum> &datum) {
            if constexpr (std::is_same_v<Source, earth::WGS> && std::is_same_v<Target, earth::ECF>) {
                return dp::Result<Target>::ok(earth::to_ecf(value));
            } else if constexpr (std::is_same_v<Source, earth::ECF> && std::is_same_v<Target, earth::WGS>) {
                return dp::Result<Target>::ok(earth::to_wgs(value));
            } else if constexpr (std::is_same_v<Source, earth::WGS> && std::is_same_v<Target, earth::UTM>) {
                return earth::to_utm(value);
            } else if constexpr (std::is_same_v<Source, earth::UTM> && std::is_same_v<Target, earth::WGS>) {
                return earth::to_wgs(value);
            } else if constexpr (std::is_same_v<Source, earth::WGS> && std::is_same_v<Target, frame::ENU>) {
                if (!datum) {
                    return dp::Result<Target>::err(dp::Error::invalid_argument("WGS->ENU requires Datum"));
                }
                return dp::Result<Target>::ok(frame::to_enu(*datum, value));
            } else if constexpr (std::is_same_v<Source, frame::ENU> && std::is_same_v<Target, earth::WGS>) {
                if (!datum) {
                    return dp::Result<Target>::err(dp::Error::invalid_argument("ENU->WGS requires Datum"));
                }
                return dp::Result<Target>::ok(frame::to_wgs(*datum, value));
            } else if constexpr (std::is_same_v<Source, earth::WGS> && std::is_same_v<Target, frame::NED>) {
                if (!datum) {
                    return dp::Result<Target>::err(dp::Error::invalid_argument("WGS->NED requires Datum"));
                }
                return dp::Result<Target>::ok(frame::to_ned(*datum, value));
            } else if constexpr (std::is_same_v<Source, frame::NED> && std::is_same_v<Target, earth::WGS>) {
                if (!datum) {
                    return dp::Result<Target>::err(dp::Error::invalid_argument("NED->WGS requires Datum"));
                }
                return dp::Result<Target>::ok(frame::to_wgs(*datum, value));
            } else if constexpr (std::is_same_v<Source, frame::ENU> && std::is_same_v<Target, frame::NED>) {
                return dp::Result<Target>::ok(frame::to_ned(value));
            } else if constexpr (std::is_same_v<Source, frame::NED> && std::is_same_v<Target, frame::ENU>) {
                return dp::Result<Target>::ok(frame::to_enu(value));
            } else {
                static_assert(sizeof(Target) == 0, "Unsupported conversion");
            }
        }

    } // namespace detail

    template <typename T> class ConvertBuilder {
      public:
        explicit ConvertBuilder(T value) : state_(dp::Result<T>::ok(std::move(value))) {}
        explicit ConvertBuilder(dp::Result<T> state) : state_(std::move(state)) {}

        ConvertBuilder &withDatum(const frame::Datum &datum) {
            datum_ = datum;
            return *this;
        }

        template <typename Target> ConvertBuilder<Target> to() const {
            if (state_.is_err()) {
                return ConvertBuilder<Target>(dp::Result<Target>::err(state_.error()), datum_);
            }
            return ConvertBuilder<Target>(detail::convert_one<Target>(state_.value(), datum_), datum_);
        }

        dp::Result<T> build() const { return state_; }

      private:
        template <typename> friend class ConvertBuilder;

        ConvertBuilder(dp::Result<T> state, dp::Optional<frame::Datum> datum)
            : state_(std::move(state)), datum_(std::move(datum)) {}

        dp::Result<T> state_;
        dp::Optional<frame::Datum> datum_{};
    };

    template <typename T> ConvertBuilder<std::decay_t<T>> convert(T &&value) {
        return ConvertBuilder<std::decay_t<T>>(std::forward<T>(value));
    }

} // namespace concord
