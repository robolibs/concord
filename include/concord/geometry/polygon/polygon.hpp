#pragma once

#include "../../core/types.hpp"
#include "../primitives/line.hpp"
#include "../bounding.hpp"
#include <cmath>
#include <cstddef>
#include <vector>

namespace concord {

    class Polygon {
      public:
        Polygon() = default;
        explicit Polygon(const std::vector<Point> &pts) : points(pts) {}

        inline void addPoint(const Point &p) { points.emplace_back(p); }

        inline std::size_t numVertices() const noexcept { return points.size(); }
        inline bool isConnected() const noexcept { return points.size() >= 3; }

        inline double perimeter() const noexcept {
            if (points.size() < 2)
                return 0.0;
            double per = 0.0;
            for (std::size_t i = 1; i < points.size(); ++i)
                per += Line(points[i - 1], points[i]).length();
            per += Line(points.back(), points.front()).length();
            return per;
        }

        inline double area() const noexcept {
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

        inline bool contains(const Point &p) const noexcept {
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

        inline Polygon from_rectangle(const float width, const float height, Size inflate = Size(1.0, 1.0, 1.0)) const {
            Polygon p;
            p.addPoint(Point{width * inflate.x / 2.0, height * inflate.y / 2.0, 0.0});
            p.addPoint(Point{width * inflate.x / 2.0, -height * inflate.y / 2.0, 0.0});
            p.addPoint(Point{-width * inflate.x / 2.0, -height * inflate.y / 2.0, 0.0});
            p.addPoint(Point{-width * inflate.x / 2.0, height * inflate.y / 2.0, 0.0});
            return p;
        }

        inline Polygon from_vector(std::vector<Point> pts) {
            Polygon p;
            for (auto &pt : pts) {
                p.addPoint(pt);
            }
            return p;
        }

        inline Polygon from_rectangle(Size size, Size inflate = Size(1.0, 1.0, 1.0)) const {
            return from_rectangle(size.x, size.y, inflate);
        }

        inline Bound get_obb() const {
            if (points.empty()) {
                return Bound();
            }

            // --- 1) Compute centroid-based orientation (same as before) ---
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

            // --- 2) Rotate all points into this frame and track min/max ---
            double minRotX = std::numeric_limits<double>::infinity();
            double maxRotX = -std::numeric_limits<double>::infinity();
            double minRotY = std::numeric_limits<double>::infinity();
            double maxRotY = -std::numeric_limits<double>::infinity();

            for (const auto &p : points) {
                double x = p.x;
                double y = p.y;
                // rotate (x,y) by +orientation_rad
                double rotX = x * cosO + y * sinO;
                double rotY = -x * sinO + y * cosO;

                minRotX = std::min(minRotX, rotX);
                maxRotX = std::max(maxRotX, rotX);
                minRotY = std::min(minRotY, rotY);
                maxRotY = std::max(maxRotY, rotY);
            }

            // --- 3) Compute width/height in rotated frame ---
            double width = maxRotX - minRotX;
            double height = maxRotY - minRotY;

            // --- 4) Compute center in rotated coords, then un-rotate back ---
            double centerRotX = 0.5 * (minRotX + maxRotX);
            double centerRotY = 0.5 * (minRotY + maxRotY);

            // inverse rotation by -orientation_rad
            double centerX = centerRotX * cosO - centerRotY * sinO;
            double centerY = centerRotX * sinO + centerRotY * cosO;

            // --- 5) Build and return the final Bound ---
            concord::Point center_pt(centerX, centerY, 0.0);
            concord::Euler euler(0.0, 0.0, orientation_rad);
            concord::Pose pose(center_pt, euler);

            concord::Size size(width, height, 0.0);
            return concord::Bound(pose, size);
        }

        inline auto begin() noexcept { return points.begin(); }
        inline auto end() noexcept { return points.end(); }
        inline auto begin() const noexcept { return points.begin(); }
        inline auto end() const noexcept { return points.end(); }

        inline const std::vector<Point> &getPoints() const noexcept { return points; }
        
        inline AABB getAABB() const {
            return AABB::fromPoints(points);
        }

        inline bool operator==(const Polygon& other) const {
            return points == other.points;
        }

        inline bool operator!=(const Polygon& other) const {
            return !(*this == other);
        }

      private:
        std::vector<Point> points;
    };

} // namespace concord
