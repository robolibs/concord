#include "concord/geometry/primitives/circle.hpp"
#include <cmath>

namespace concord {

    Circle::Circle(const Point &center_, double radius_) : center(center_), radius(radius_) {}

    double Circle::area() const noexcept { return M_PI * radius * radius; }

    double Circle::circumference() const noexcept { return 2 * M_PI * radius; }

    const Point &Circle::getCenter() const noexcept { return center; }

    void Circle::setCenter(const Point &c) noexcept { center = c; }

    bool Circle::contains(const Point &p) const noexcept {
        return std::sqrt(std::pow(p.x - center.x, 2) + std::pow(p.y - center.y, 2)) < radius;
    }

    std::vector<Point> Circle::as_polygon(int n) const {
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

    double Circle::getRadius() const noexcept { return radius; }

    void Circle::setRadius(double r) noexcept { radius = r; }

    bool Circle::operator==(const Circle &other) const { return center == other.center && radius == other.radius; }

    bool Circle::operator!=(const Circle &other) const { return !(*this == other); }

} // namespace concord
