#pragma once

#include "../geometry/bounding.hpp"
#include "../geometry/polygon/polygon.hpp"
#include "../geometry/primitives/line.hpp"
#include "../types/point.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <vector>

namespace concord {

    namespace spatial {

        // Distance calculations
        inline double distance(const Point &a, const Point &b) {
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            double dz = a.z - b.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        inline double distance2D(const Point &a, const Point &b) {
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            return std::sqrt(dx * dx + dy * dy);
        }

        inline double distanceSquared(const Point &a, const Point &b) {
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            double dz = a.z - b.z;
            return dx * dx + dy * dy + dz * dz;
        }

        // Point-to-line distance
        inline double distanceToLine(const Point &point, const Line &line) {
            const auto &p1 = line.getStart();
            const auto &p2 = line.getEnd();
            const auto &p = point;

            double A = p.x - p1.x;
            double B = p.y - p1.y;
            double C = p2.x - p1.x;
            double D = p2.y - p1.y;

            double dot = A * C + B * D;
            double len_sq = C * C + D * D;

            if (len_sq < 1e-10) {
                // Line is actually a point
                return distance2D(point, line.getStart());
            }

            double param = dot / len_sq;

            double xx, yy;
            if (param < 0) {
                xx = p1.x;
                yy = p1.y;
            } else if (param > 1) {
                xx = p2.x;
                yy = p2.y;
            } else {
                xx = p1.x + param * C;
                yy = p1.y + param * D;
            }

            double dx = p.x - xx;
            double dy = p.y - yy;
            return std::sqrt(dx * dx + dy * dy);
        }

        // Line-line intersection
        inline bool lineIntersection(const Line &line1, const Line &line2, Point &result) {
            const auto &p1 = line1.getStart();
            const auto &p2 = line1.getEnd();
            const auto &p3 = line2.getStart();
            const auto &p4 = line2.getEnd();

            double x1 = p1.x, y1 = p1.y;
            double x2 = p2.x, y2 = p2.y;
            double x3 = p3.x, y3 = p3.y;
            double x4 = p4.x, y4 = p4.y;

            double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

            if (std::abs(denom) < 1e-10) {
                return false; // Lines are parallel
            }

            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
            double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                double ix = x1 + t * (x2 - x1);
                double iy = y1 + t * (y2 - y1);
                result = Point{ix, iy, 0.0};
                return true;
            }

            return false;
        }

        // Polygon operations
        inline bool isClockwise(const Polygon &polygon) {
            if (polygon.numVertices() < 3)
                return false;

            double sum = 0.0;
            const auto &points = polygon.getPoints();

            for (size_t i = 0; i < points.size(); ++i) {
                const auto &p1 = points[i];
                const auto &p2 = points[(i + 1) % points.size()];
                sum += (p2.x - p1.x) * (p2.y + p1.y);
            }

            return sum > 0.0;
        }

        inline Polygon reverse(const Polygon &polygon) {
            auto points = polygon.getPoints();
            std::reverse(points.begin(), points.end());
            return Polygon{points};
        }

        // Convex hull using Graham scan
        inline Polygon convexHull(std::vector<Point> points) {
            if (points.size() < 3) {
                return Polygon{points};
            }

            // Find bottom-most point (or left most in case of tie)
            auto bottom = std::min_element(points.begin(), points.end(), [](const Point &a, const Point &b) {
                return a.y < b.y || (a.y == b.y && a.x < b.x);
            });

            std::swap(*bottom, points[0]);
            Point pivot = points[0];

            // Sort points by polar angle with respect to pivot
            std::sort(points.begin() + 1, points.end(), [&pivot](const Point &a, const Point &b) {
                double dx1 = a.x - pivot.x;
                double dy1 = a.y - pivot.y;
                double dx2 = b.x - pivot.x;
                double dy2 = b.y - pivot.y;

                double cross = dx1 * dy2 - dy1 * dx2;
                if (std::abs(cross) < 1e-10) {
                    // Collinear, sort by distance
                    return dx1 * dx1 + dy1 * dy1 < dx2 * dx2 + dy2 * dy2;
                }
                return cross > 0;
            });

            std::vector<Point> hull;
            for (const auto &point : points) {
                while (hull.size() >= 2) {
                    const auto &p1 = hull[hull.size() - 2];
                    const auto &p2 = hull[hull.size() - 1];
                    const auto &p3 = point;

                    double cross = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

                    if (cross <= 0) {
                        hull.pop_back();
                    } else {
                        break;
                    }
                }
                hull.push_back(point);
            }

            return Polygon{hull};
        }

        // Polygon simplification using Douglas-Peucker algorithm
        inline Polygon simplify(const Polygon &polygon, double tolerance) {
            const auto &points = polygon.getPoints();
            if (points.size() <= 2) {
                return polygon;
            }

            std::function<std::vector<Point>(const std::vector<Point> &, double)> douglasPeucker;
            douglasPeucker = [&](const std::vector<Point> &pts, double tol) -> std::vector<Point> {
                if (pts.size() <= 2) {
                    return pts;
                }

                // Find point with maximum distance from line
                double max_dist = 0.0;
                size_t max_index = 0;

                Line line{pts[0], pts.back()};

                for (size_t i = 1; i < pts.size() - 1; ++i) {
                    double dist = distanceToLine(pts[i], line);
                    if (dist > max_dist) {
                        max_dist = dist;
                        max_index = i;
                    }
                }

                if (max_dist > tol) {
                    // Recursively simplify sub-segments
                    std::vector<Point> pts1(pts.begin(), pts.begin() + max_index + 1);
                    std::vector<Point> pts2(pts.begin() + max_index, pts.end());

                    auto result1 = douglasPeucker(pts1, tol);
                    auto result2 = douglasPeucker(pts2, tol);

                    // Combine results (remove duplicate point)
                    result1.insert(result1.end(), result2.begin() + 1, result2.end());
                    return result1;
                } else {
                    // All points can be approximated by line
                    return {pts[0], pts.back()};
                }
            };

            auto simplified = douglasPeucker(points, tolerance);
            return Polygon{simplified};
        }

        // Buffer/offset operations
        inline Polygon buffer(const Polygon &polygon, double distance, int /* segments */ = 16) {
            // Simplified buffer implementation
            // In practice, you'd want a more robust implementation using CGAL or similar

            std::vector<Point> buffered_points;
            const auto &points = polygon.getPoints();

            for (size_t i = 0; i < points.size(); ++i) {
                const auto &curr = points[i];
                const auto &next = points[(i + 1) % points.size()];
                const auto &prev = points[(i + points.size() - 1) % points.size()];

                // Calculate normal vectors
                double dx1 = curr.x - prev.x;
                double dy1 = curr.y - prev.y;
                double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
                if (len1 > 1e-10) {
                    dx1 /= len1;
                    dy1 /= len1;
                }

                double dx2 = next.x - curr.x;
                double dy2 = next.y - curr.y;
                double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
                if (len2 > 1e-10) {
                    dx2 /= len2;
                    dy2 /= len2;
                }

                // Average normal (simplified)
                double nx = -(dy1 + dy2) * 0.5;
                double ny = (dx1 + dx2) * 0.5;
                double nlen = std::sqrt(nx * nx + ny * ny);
                if (nlen > 1e-10) {
                    nx /= nlen;
                    ny /= nlen;
                }

                Point buffered_point;
                buffered_point.x = curr.x + nx * distance;
                buffered_point.y = curr.y + ny * distance;
                buffered_point.z = curr.z;

                buffered_points.push_back(buffered_point);
            }

            return Polygon{buffered_points};
        }

        // Spatial clustering (simple distance-based)
        inline std::vector<std::vector<Point>> cluster(const std::vector<Point> &points, double max_distance) {
            std::vector<std::vector<Point>> clusters;
            std::vector<bool> visited(points.size(), false);

            for (size_t i = 0; i < points.size(); ++i) {
                if (visited[i])
                    continue;

                std::vector<Point> cluster;
                std::vector<size_t> stack = {i};

                while (!stack.empty()) {
                    size_t current = stack.back();
                    stack.pop_back();

                    if (visited[current])
                        continue;
                    visited[current] = true;
                    cluster.push_back(points[current]);

                    // Find neighbors
                    for (size_t j = 0; j < points.size(); ++j) {
                        if (!visited[j] && distance(points[current], points[j]) <= max_distance) {
                            stack.push_back(j);
                        }
                    }
                }

                clusters.push_back(cluster);
            }

            return clusters;
        }

    } // namespace spatial

} // namespace concord
