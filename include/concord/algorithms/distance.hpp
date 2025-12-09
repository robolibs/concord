#pragma once

#include "../geometry/primitives/line.hpp"
#include "../types/point.hpp"
#include <cmath>

namespace concord {
    namespace algorithms {
        namespace distance {

            // Basic distance calculations
            double euclidean(const Point &a, const Point &b);
            double euclidean2D(const Point &a, const Point &b);
            double euclidean_squared(const Point &a, const Point &b);

            // Manhattan distance
            double manhattan(const Point &a, const Point &b);

            // Point-to-line distance
            double point_to_line(const Point &point, const Line &line);

        } // namespace distance
    } // namespace algorithms
} // namespace concord

