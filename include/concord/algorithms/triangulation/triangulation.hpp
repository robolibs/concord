#pragma once

#include "../../types/point.hpp"
#include <vector>

namespace concord {
    namespace algorithms {
        namespace triangulation {

            struct Triangle {
                Point a, b, c;
                Triangle(const Point &a, const Point &b, const Point &c) : a(a), b(b), c(c) {}
            };

            // Delaunay triangulation
            std::vector<Triangle> delaunay(const std::vector<Point> &points);

            // Constrained Delaunay triangulation
            std::vector<Triangle> constrained_delaunay(const std::vector<Point> &points,
                                                       const std::vector<std::pair<int, int>> &constraints);

            // Ear clipping for simple polygons
            std::vector<Triangle> ear_clipping(const std::vector<Point> &polygon);

        } // namespace triangulation
    } // namespace algorithms
} // namespace concord
