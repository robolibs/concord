#pragma once

#include "../../types/point.hpp"
#include <cmath>

namespace concord {

    class Line {
      public:
        Line() = default;
        Line(const Point &s, const Point &e);

        double length() const noexcept;

        const Point &getStart() const noexcept;
        const Point &getEnd() const noexcept;

        void setStart(const Point &s) noexcept;
        void setEnd(const Point &e) noexcept;

        bool operator==(const Line &other) const;
        bool operator!=(const Line &other) const;

      private:
        Point start;
        Point end;
    };

} // namespace concord
