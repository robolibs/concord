#pragma once

#include "../../geometry/primitives/line.hpp"
#include "../../types/point.hpp"
#include <cmath>

namespace concord {
    namespace algorithms {
        namespace distance {

            // Basic distance calculations
            inline double euclidean(const Point &a, const Point &b) {
                double dx = a.x - b.x;
                double dy = a.y - b.y;
                double dz = a.z - b.z;
                return std::sqrt(dx * dx + dy * dy + dz * dz);
            }

            inline double euclidean2D(const Point &a, const Point &b) {
                double dx = a.x - b.x;
                double dy = a.y - b.y;
                return std::sqrt(dx * dx + dy * dy);
            }

            inline double euclidean_squared(const Point &a, const Point &b) {
                double dx = a.x - b.x;
                double dy = a.y - b.y;
                double dz = a.z - b.z;
                return dx * dx + dy * dy + dz * dz;
            }

            // Manhattan distance
            inline double manhattan(const Point &a, const Point &b) {
                return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
            }

            // Point-to-line distance
            inline double point_to_line(const Point & /*point*/, const Line & /*line*/) {
                // Implementation placeholder
                return 0.0;
            }

        } // namespace distance
    } // namespace algorithms
} // namespace concord
