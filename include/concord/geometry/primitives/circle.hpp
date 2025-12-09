#pragma once

#include "../../types/point.hpp"
#include <cmath>
#include <vector>

namespace concord {

    class Circle {
      public:
        Circle() = default;
        Circle(const Point &center_, double radius_);

        double area() const noexcept;
        double circumference() const noexcept;

        const Point &getCenter() const noexcept;
        void setCenter(const Point &c) noexcept;

        bool contains(const Point &p) const noexcept;

        std::vector<Point> as_polygon(int n = 100) const;

        double getRadius() const noexcept;
        void setRadius(double r) noexcept;

        bool operator==(const Circle &other) const;
        bool operator!=(const Circle &other) const;

      private:
        Point center;
        double radius;
    };

} // namespace concord
