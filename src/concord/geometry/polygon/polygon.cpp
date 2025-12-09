#include "concord/geometry/polygon/polygon.hpp"
#include "concord/geometry/primitives/line.hpp"
#include <cmath>
#include <limits>

namespace concord {

    Polygon::Polygon(const std::vector<Point> &pts) : points(pts) {}

    void Polygon::addPoint(const Point &p) { 
        points.emplace_back(p); 
    }

    std::size_t Polygon::numVertices() const noexcept { 
        return points.size(); 
    }

    bool Polygon::isConnected() const noexcept { 
        return points.size() >= 3; 
    }

    double Polygon::perimeter() const noexcept {
        if (points.size() < 2)
            return 0.0;
        double per = 0.0;
        for (std::size_t i = 1; i < points.size(); ++i)
            per += Line(points[i - 1], points[i]).length();
        per += Line(points.back(), points.front()).length();
        return per;
    }

    double Polygon::area() const noexcept {
        if (points.size() < 3)
            return 0.0;
        double a = 0.0;
        for (std::size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
            const auto &pi = points[i];
            const auto &pj = points[j];
            a += (pj.x + pi.x) * (pj.y - pi.y);
        }
        return std::abs(a * 0.5);
    }

    bool Polygon::contains(const Point &p) const noexcept {
        if (points.size() < 3)
            return false;
        bool c = false;
        for (std::size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
            const auto &pi = points[i];
            const auto &pj = points[j];
            if (((pi.y > p.y) != (pj.y > p.y)) && (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x))
                c = !c;
        }
        return c;
    }

    Polygon Polygon::from_rectangle(const float width, const float height, Size inflate) const {
        Polygon p;
        p.addPoint(Point{width * inflate.x / 2.0, height * inflate.y / 2.0, 0.0});
        p.addPoint(Point{width * inflate.x / 2.0, -height * inflate.y / 2.0, 0.0});
        p.addPoint(Point{-width * inflate.x / 2.0, -height * inflate.y / 2.0, 0.0});
        p.addPoint(Point{-width * inflate.x / 2.0, height * inflate.y / 2.0, 0.0});
        return p;
    }

    Polygon Polygon::from_vector(std::vector<Point> pts) {
        Polygon p;
        for (auto &pt : pts) {
            p.addPoint(pt);
        }
        return p;
    }

    Polygon Polygon::from_rectangle(Size size, Size inflate) const {
        return from_rectangle(size.x, size.y, inflate);
    }

    Bound Polygon::get_obb() const {
        if (points.empty()) {
            return Bound();
        }

        double sumX = 0.0, sumY = 0.0;
        for (const auto &p : points) {
            sumX += p.x;
            sumY += p.y;
        }
        double centroidX = sumX / points.size();
        double centroidY = sumY / points.size();

        const auto &first = points[0];
        double orientation_rad = std::atan2(centroidY - first.y, centroidX - first.x);
        double cosO = std::cos(orientation_rad);
        double sinO = std::sin(orientation_rad);

        double minRotX = std::numeric_limits<double>::infinity();
        double maxRotX = -std::numeric_limits<double>::infinity();
        double minRotY = std::numeric_limits<double>::infinity();
        double maxRotY = -std::numeric_limits<double>::infinity();

        for (const auto &p : points) {
            double x = p.x;
            double y = p.y;
            double rotX = x * cosO + y * sinO;
            double rotY = -x * sinO + y * cosO;

            minRotX = std::min(minRotX, rotX);
            maxRotX = std::max(maxRotX, rotX);
            minRotY = std::min(minRotY, rotY);
            maxRotY = std::max(maxRotY, rotY);
        }

        double width = maxRotX - minRotX;
        double height = maxRotY - minRotY;

        double centerRotX = 0.5 * (minRotX + maxRotX);
        double centerRotY = 0.5 * (minRotY + maxRotY);

        double centerX = centerRotX * cosO - centerRotY * sinO;
        double centerY = centerRotX * sinO + centerRotY * cosO;

        concord::Point center_pt(centerX, centerY, 0.0);
        concord::Euler euler(0.0, 0.0, orientation_rad);
        concord::Pose pose(center_pt, euler);

        concord::Size size(width, height, 0.0);
        return concord::Bound(pose, size);
    }

    auto Polygon::begin() noexcept { 
        return points.begin(); 
    }

    auto Polygon::end() noexcept { 
        return points.end(); 
    }

    auto Polygon::begin() const noexcept { 
        return points.begin(); 
    }

    auto Polygon::end() const noexcept { 
        return points.end(); 
    }

    const std::vector<Point> &Polygon::getPoints() const noexcept { 
        return points; 
    }

    AABB Polygon::getAABB() const { 
        return AABB::fromPoints(points); 
    }

    bool Polygon::operator==(const Polygon &other) const { 
        return points == other.points; 
    }

    bool Polygon::operator!=(const Polygon &other) const { 
        return !(*this == other); 
    }

} // namespace concord
