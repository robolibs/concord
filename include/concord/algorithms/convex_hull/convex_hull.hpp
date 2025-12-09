#pragma once

#include "../../types/point.hpp"
#include <vector>

namespace concord {
    namespace algorithms {
        namespace convex_hull {

            // Graham scan algorithm for 2D convex hull
            std::vector<Point> graham_scan(const std::vector<Point> &points);

            // QuickHull algorithm for 2D/3D convex hull
            std::vector<Point> quickhull(const std::vector<Point> &points);

            // Gift wrapping algorithm (Jarvis march)
            std::vector<Point> gift_wrapping(const std::vector<Point> &points);

        } // namespace convex_hull
    } // namespace algorithms
} // namespace concord
