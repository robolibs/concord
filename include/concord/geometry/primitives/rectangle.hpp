#pragma once

#include "../../types/bound.hpp"
#include "../../types/euler.hpp"
#include "../../types/point.hpp"
#include "line.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <tuple>
#include <vector>

namespace concord {

    class Rectangle {
      public:
        Rectangle() = default;
        Rectangle(const Point &tl, const Point &tr, const Point &bl, const Point &br);

        double area() const noexcept;
        double perimeter() const noexcept;
        bool contains(const Point &p) const noexcept;

        void from_pointvec(std::array<Point, 4> points);

        const Point &getTopLeft() const noexcept;
        const Point &getTopRight() const noexcept;
        const Point &getBottomLeft() const noexcept;
        const Point &getBottomRight() const noexcept;

        std::array<Point, 4> get_corners() const noexcept;

        static Rectangle outer_rectangle(const std::vector<Bound> &bounds);

        bool operator==(const Rectangle &other) const;
        bool operator!=(const Rectangle &other) const;

      private:
        Point top_left;
        Point top_right;
        Point bottom_left;
        Point bottom_right;
        Euler orientation;
    };

} // namespace concord
