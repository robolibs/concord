
#pragma once

#include "../../core/types.hpp" // for Datum, Point, ENU, Euler, Bound
#include "line.hpp"             // for Line
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
        Rectangle(const Point &tl, const Point &tr, const Point &bl, const Point &br)
            : top_left(tl), top_right(tr), bottom_left(bl), bottom_right(br) {}

        inline double area() const noexcept {
            double width = Line(top_left, top_right).length();
            double height = Line(top_left, bottom_left).length();
            return width * height;
        }

        inline double perimeter() const noexcept {
            double w = Line(top_left, top_right).length();
            double h = Line(top_left, bottom_left).length();
            return 2 * (w + h);
        }

        inline bool contains(const Point &p) const noexcept {
            return (p.x >= top_left.x && p.x <= top_right.x && p.y >= top_left.y && p.y <= bottom_left.y);
        }

        inline void from_pointvec(std::array<Point, 4> points) {
            top_left = points[0];
            top_right = points[1];
            bottom_left = points[2];
            bottom_right = points[3];
        }

        inline const Point &getTopLeft() const noexcept { return top_left; }
        inline const Point &getTopRight() const noexcept { return top_right; }
        inline const Point &getBottomLeft() const noexcept { return bottom_left; }
        inline const Point &getBottomRight() const noexcept { return bottom_right; }

        inline std::array<Point, 4> get_corners() const noexcept {
            return {top_left, top_right, bottom_right, bottom_left};
        }

        static inline Rectangle outer_rectangle(const std::vector<Bound> &bounds) {
            if (bounds.empty()) {
                return Rectangle();
            }

            struct P2 {
                double x, y;
                bool operator<(P2 const &o) const { return x < o.x || (x == o.x && y < o.y); }
            };
            auto cross = [&](P2 const &O, P2 const &A, P2 const &B) {
                return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
            };

            // --- convex hull (Andrew’s monotone chain) ---
            auto convexHull = [&](std::vector<P2> pts) {
                size_t n = pts.size(), k = 0;
                if (n < 3)
                    return pts;
                std::sort(pts.begin(), pts.end());
                std::vector<P2> H(2 * n);
                for (size_t i = 0; i < n; ++i) {
                    while (k >= 2 && cross(H[k - 2], H[k - 1], pts[i]) <= 0)
                        --k;
                    H[k++] = pts[i];
                }
                for (size_t i = n - 2, t = k + 1; i < n; --i) {
                    while (k >= t && cross(H[k - 2], H[k - 1], pts[i]) <= 0)
                        --k;
                    H[k++] = pts[i];
                }
                H.resize(k - 1);
                return H;
            };

            // --- rotating-calipers for min-area rectangle ---
            auto minAreaRect = [&](std::vector<P2> const &hull) {
                double bestA = std::numeric_limits<double>::infinity();
                double bx = 0, by = 0, bw = 0, bh = 0, ba = 0;
                size_t m = hull.size();
                for (size_t i = 0; i < m; ++i) {
                    size_t j = (i + 1) % m;
                    double dx = hull[j].x - hull[i].x;
                    double dy = hull[j].y - hull[i].y;
                    double ang = std::atan2(dy, dx);
                    double c = std::cos(ang), s = std::sin(ang);

                    double minX = std::numeric_limits<double>::infinity();
                    double maxX = -std::numeric_limits<double>::infinity();
                    double minY = std::numeric_limits<double>::infinity();
                    double maxY = -std::numeric_limits<double>::infinity();

                    for (auto &p : hull) {
                        double ux = p.x * c + p.y * s;
                        double uy = -p.x * s + p.y * c;
                        minX = std::min(minX, ux);
                        maxX = std::max(maxX, ux);
                        minY = std::min(minY, uy);
                        maxY = std::max(maxY, uy);
                    }

                    double w = maxX - minX;
                    double h_val = maxY - minY;
                    double area = w * h_val;
                    if (area < bestA) {
                        bestA = area;
                        double cenXr = 0.5 * (minX + maxX);
                        double cenYr = 0.5 * (minY + maxY);
                        bx = cenXr * c - cenYr * s;
                        by = cenXr * s + cenYr * c;
                        bw = w;
                        bh = h_val;
                        ba = ang;
                    }
                }
                return std::tuple<double, double, double, double, double>{bx, by, bw, bh, ba};
            };

            // --- 1) Collect ALL corners of every Bound ---
            std::vector<P2> pts;
            pts.reserve(bounds.size() * 4);
            for (auto const &b : bounds) {
                double cx = b.pose.point.x;
                double cy = b.pose.point.y;
                double ang = b.pose.angle.yaw;
                double c = std::cos(ang);
                double s = std::sin(ang);
                double hw = b.size.x * 0.5;
                double hh = b.size.y * 0.5;

                P2 local[4] = {{+hw, +hh}, {-hw, +hh}, {-hw, -hh}, {+hw, -hh}};
                for (auto const &l : local) {
                    pts.push_back({cx + l.x * c - l.y * s, cy + l.x * s + l.y * c});
                }
            }

            // --- 2) Hull, 3) Min-area rect, 4) Rebuild Rectangle ---
            auto hull = convexHull(std::move(pts));
            auto [rx, ry, rw, rh, rr] = minAreaRect(hull);

            double hw = rw * 0.5, hh = rh * 0.5;
            double cc = std::cos(rr), ss = std::sin(rr);
            auto mkPt = [&](double lx, double ly) {
                double x = rx + lx * cc - ly * ss;
                double y = ry + lx * ss + ly * cc;
                return Point{x, y, 0.0};
            };

            Point tl = mkPt(-hw, +hh);
            Point tr = mkPt(+hw, +hh);
            Point bl = mkPt(-hw, -hh);
            Point br = mkPt(+hw, -hh);

            return Rectangle(tl, tr, bl, br);
        }

        inline bool operator==(const Rectangle& other) const {
            return top_left == other.top_left && top_right == other.top_right && 
                   bottom_left == other.bottom_left && bottom_right == other.bottom_right &&
                   orientation == other.orientation;
        }

        inline bool operator!=(const Rectangle& other) const {
            return !(*this == other);
        }

      private:
        Point top_left;
        Point top_right;
        Point bottom_left;
        Point bottom_right;
        Euler orientation;
    };

} // namespace concord
