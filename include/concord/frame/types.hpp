#pragma once

#include <datapod/datapod.hpp>
#include <tuple>

namespace concord::frame {

    struct ENU {
        dp::Point p; // east,north,up meters

        auto members() noexcept { return std::tie(p); }
        auto members() const noexcept { return std::tie(p); }

        bool is_set() const noexcept { return p.is_set(); }
    };

    struct NED {
        dp::Point p; // north,east,down meters

        auto members() noexcept { return std::tie(p); }
        auto members() const noexcept { return std::tie(p); }

        bool is_set() const noexcept { return p.is_set(); }
    };

} // namespace concord::frame
