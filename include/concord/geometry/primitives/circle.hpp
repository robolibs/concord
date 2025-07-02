#pragma once

#include "../../core/types.hpp"
#include <cmath>

namespace concord {

    class Circle {
      public:
        Circle() = default;
        Circle(const Point &center_, double radius_) : center(center_), radius(radius_) {}

        inline double area() const noexcept { return M_PI * radius * radius; }
        inline double circumference() const noexcept { return 2 * M_PI * radius; }

        inline const Point &getCenter() const noexcept { return center; }
        inline void setCenter(const Point &c) noexcept { center = c; }

        inline bool contains(const Point &p) const noexcept {
            return std::sqrt(std::pow(p.x - center.x, 2) + std::pow(p.y - center.y, 2)) < radius;
        }

        inline std::vector<Point> as_polygon(int n = 100) const {
            std::vector<Point> points;
            double theta = 2 * M_PI / n;
            for (int i = 0; i < n; i++) {
                double x = center.x + radius * std::cos(theta * i);
                double y = center.y + radius * std::sin(theta * i);
                Point p{x, y, 0.0};
                points.push_back(p);
            }
            return points;
        }

        inline double getRadius() const noexcept { return radius; }
        inline void setRadius(double r) noexcept { radius = r; }

        inline bool operator==(const Circle& other) const {
            return center == other.center && radius == other.radius;
        }

        inline bool operator!=(const Circle& other) const {
            return !(*this == other);
        }

      private:
        Point center;
        double radius;
    };

} // namespace concord
