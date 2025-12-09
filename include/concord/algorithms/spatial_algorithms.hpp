#pragma once

#include "../geometry/bounding.hpp"
#include "../geometry/polygon/polygon.hpp"
#include "../geometry/primitives/line.hpp"
#include "../types/point.hpp"
#include <vector>

namespace concord {

    namespace spatial {

        // Distance calculations
        double distance(const Point &a, const Point &b);
        double distance2D(const Point &a, const Point &b);
        double distanceSquared(const Point &a, const Point &b);

        // Point-to-line distance
        double distanceToLine(const Point &point, const Line &line);

        // Line-line intersection
        bool lineIntersection(const Line &line1, const Line &line2, Point &result);

        // Polygon operations
        bool isClockwise(const Polygon &polygon);
        Polygon reverse(const Polygon &polygon);

        // Convex hull using Graham scan
        Polygon convexHull(std::vector<Point> points);

        // Polygon simplification using Douglas-Peucker algorithm
        Polygon simplify(const Polygon &polygon, double tolerance);

        // Buffer/offset operations
        Polygon buffer(const Polygon &polygon, double distance, int segments = 16);

        // Spatial clustering (simple distance-based)
        std::vector<std::vector<Point>> cluster(const std::vector<Point> &points, double max_distance);

    } // namespace spatial

} // namespace concord
