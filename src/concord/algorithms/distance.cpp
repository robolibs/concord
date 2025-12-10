#include "concord/algorithms/distance.hpp"

#include <cmath>

namespace concord {
    namespace algorithms {
        namespace distance {

            double euclidean(const Point &a, const Point &b) {
                double dx = a.x - b.x;
                double dy = a.y - b.y;
                double dz = a.z - b.z;
                return std::sqrt(dx * dx + dy * dy + dz * dz);
            }

            double euclidean2D(const Point &a, const Point &b) {
                double dx = a.x - b.x;
                double dy = a.y - b.y;
                return std::sqrt(dx * dx + dy * dy);
            }

            double euclidean_squared(const Point &a, const Point &b) {
                double dx = a.x - b.x;
                double dy = a.y - b.y;
                double dz = a.z - b.z;
                return dx * dx + dy * dy + dz * dz;
            }

            double manhattan(const Point &a, const Point &b) {
                return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
            }

            double point_to_line(const Point & /*point*/, const Line & /*line*/) {
                // Implementation placeholder (kept consistent with previous inline stub)
                return 0.0;
            }

        } // namespace distance
    } // namespace algorithms
} // namespace concord
