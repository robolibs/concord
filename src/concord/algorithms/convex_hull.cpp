#include "concord/algorithms/convex_hull.hpp"

#include <algorithm>
#include <cmath>

namespace concord {
    namespace algorithms {
        namespace convex_hull {

            namespace {
                std::vector<Point> graham_scan_impl(std::vector<Point> points) {
                    if (points.size() < 3) {
                        return points;
                    }

                    auto bottom = std::min_element(points.begin(), points.end(), [](const Point &a, const Point &b) {
                        return a.y < b.y || (a.y == b.y && a.x < b.x);
                    });

                    std::swap(*bottom, points[0]);
                    Point pivot = points[0];

                    std::sort(points.begin() + 1, points.end(), [&pivot](const Point &a, const Point &b) {
                        double dx1 = a.x - pivot.x;
                        double dy1 = a.y - pivot.y;
                        double dx2 = b.x - pivot.x;
                        double dy2 = b.y - pivot.y;

                        double cross = dx1 * dy2 - dy1 * dx2;
                        if (std::abs(cross) < 1e-10) {
                            return dx1 * dx1 + dy1 * dy1 < dx2 * dx2 + dy2 * dy2;
                        }
                        return cross > 0;
                    });

                    std::vector<Point> hull;
                    for (const auto &p : points) {
                        while (hull.size() >= 2) {
                            const auto &p1 = hull[hull.size() - 2];
                            const auto &p2 = hull[hull.size() - 1];

                            double cross = (p2.x - p1.x) * (p.y - p1.y) - (p2.y - p1.y) * (p.x - p1.x);
                            if (cross <= 0) {
                                hull.pop_back();
                            } else {
                                break;
                            }
                        }
                        hull.push_back(p);
                    }

                    return hull;
                }
            } // namespace

            std::vector<Point> graham_scan(const std::vector<Point> &points) { return graham_scan_impl(points); }

            std::vector<Point> quickhull(const std::vector<Point> &points) {
                // For now, reuse Graham scan as a simple convex hull implementation.
                return graham_scan_impl(points);
            }

            std::vector<Point> gift_wrapping(const std::vector<Point> &points) {
                if (points.size() < 3) {
                    return points;
                }

                std::vector<Point> hull;
                size_t leftmost = 0;
                for (size_t i = 1; i < points.size(); ++i) {
                    if (points[i].x < points[leftmost].x) {
                        leftmost = i;
                    }
                }

                size_t p = leftmost;
                do {
                    hull.push_back(points[p]);
                    size_t q = (p + 1) % points.size();
                    for (size_t r = 0; r < points.size(); ++r) {
                        double cross = (points[q].x - points[p].x) * (points[r].y - points[p].y) -
                                       (points[q].y - points[p].y) * (points[r].x - points[p].x);
                        if (cross < 0) {
                            q = r;
                        }
                    }
                    p = q;
                } while (p != leftmost);

                return hull;
            }

        } // namespace convex_hull
    } // namespace algorithms
} // namespace concord
