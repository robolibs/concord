#include "concord/geometry/primitives/triangle.hpp"
#include <algorithm>
#include <cmath>

namespace concord {

    Triangle::Triangle(const Point &a_, const Point &b_, const Point &c_) : a(a_), b(b_), c(c_) {}

    Triangle Triangle::from_vision(const Point &apex, double heading, double range, double half_angle) {
        // Calculate the two far corners of the vision triangle
        double left_angle = heading + half_angle;
        double right_angle = heading - half_angle;

        Point left_corner{apex.x + range * std::cos(left_angle), apex.y + range * std::sin(left_angle), apex.z};

        Point right_corner{apex.x + range * std::cos(right_angle), apex.y + range * std::sin(right_angle), apex.z};

        return Triangle(apex, left_corner, right_corner);
    }

    double Triangle::sign(const Point &p1, const Point &p2, const Point &p3) noexcept {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

    bool Triangle::contains(const Point &p) const noexcept {
        // Barycentric coordinate method for point-in-triangle test
        double d1 = sign(p, a, b);
        double d2 = sign(p, b, c);
        double d3 = sign(p, c, a);

        bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    bool Triangle::intersects_circle(const Point &center, double radius) const noexcept {
        // First check if center is inside triangle
        if (contains(center)) {
            return true;
        }

        // Check if any vertex is inside the circle
        auto dist_sq = [&center](const Point &p) {
            double dx = p.x - center.x;
            double dy = p.y - center.y;
            return dx * dx + dy * dy;
        };

        double r_sq = radius * radius;
        if (dist_sq(a) <= r_sq || dist_sq(b) <= r_sq || dist_sq(c) <= r_sq) {
            return true;
        }

        // Check if circle intersects any edge
        auto point_to_segment_dist_sq = [](const Point &p, const Point &v, const Point &w) {
            double l2 = (w.x - v.x) * (w.x - v.x) + (w.y - v.y) * (w.y - v.y);
            if (l2 == 0.0)
                return (p.x - v.x) * (p.x - v.x) + (p.y - v.y) * (p.y - v.y);

            double t = std::max(0.0, std::min(1.0, ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2));

            double proj_x = v.x + t * (w.x - v.x);
            double proj_y = v.y + t * (w.y - v.y);

            return (p.x - proj_x) * (p.x - proj_x) + (p.y - proj_y) * (p.y - proj_y);
        };

        if (point_to_segment_dist_sq(center, a, b) <= r_sq)
            return true;
        if (point_to_segment_dist_sq(center, b, c) <= r_sq)
            return true;
        if (point_to_segment_dist_sq(center, c, a) <= r_sq)
            return true;

        return false;
    }

    double Triangle::area() const noexcept {
        // Shoelace formula (half the absolute value of the cross product)
        return std::abs((a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2.0);
    }

    double Triangle::perimeter() const noexcept {
        auto dist = [](const Point &p1, const Point &p2) {
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            return std::sqrt(dx * dx + dy * dy);
        };
        return dist(a, b) + dist(b, c) + dist(c, a);
    }

    std::vector<Point> Triangle::get_vertices() const { return {a, b, c}; }

    const Point &Triangle::getA() const noexcept { return a; }
    const Point &Triangle::getB() const noexcept { return b; }
    const Point &Triangle::getC() const noexcept { return c; }

    void Triangle::setA(const Point &p) noexcept { a = p; }
    void Triangle::setB(const Point &p) noexcept { b = p; }
    void Triangle::setC(const Point &p) noexcept { c = p; }

    bool Triangle::operator==(const Triangle &other) const { return a == other.a && b == other.b && c == other.c; }

    bool Triangle::operator!=(const Triangle &other) const { return !(*this == other); }

} // namespace concord
