#include "concord/algorithms/triangulation.hpp"

#include <cstddef>

namespace concord {
    namespace algorithms {
        namespace triangulation {

            namespace {
                std::vector<Triangle> ear_clipping_impl(const std::vector<Point> &polygon) {
                    std::vector<Triangle> result;
                    if (polygon.size() < 3) {
                        return result;
                    }

                    std::vector<Point> verts = polygon;
                    while (verts.size() >= 3) {
                        bool ear_found = false;
                        for (size_t i = 0; i < verts.size(); ++i) {
                            const Point &prev = verts[(i + verts.size() - 1) % verts.size()];
                            const Point &curr = verts[i];
                            const Point &next = verts[(i + 1) % verts.size()];

                            double cross = (curr.x - prev.x) * (next.y - prev.y) -
                                           (curr.y - prev.y) * (next.x - prev.x);
                            if (cross <= 0) {
                                continue;
                            }

                            Triangle tri(prev, curr, next);
                            result.push_back(tri);
                            verts.erase(verts.begin() + static_cast<std::ptrdiff_t>(i));
                            ear_found = true;
                            break;
                        }

                        if (!ear_found) {
                            break;
                        }
                    }

                    return result;
                }
            } // namespace

            std::vector<Triangle> delaunay(const std::vector<Point> &points) {
                // Placeholder: return an empty triangulation for now.
                return {};
            }

            std::vector<Triangle> constrained_delaunay(const std::vector<Point> &points,
                                                       const std::vector<std::pair<int, int>> & /*constraints*/) {
                // Placeholder: unconstrained ear clipping on given points interpreted as polygon.
                return ear_clipping_impl(points);
            }

            std::vector<Triangle> ear_clipping(const std::vector<Point> &polygon) {
                return ear_clipping_impl(polygon);
            }

        } // namespace triangulation
    }     // namespace algorithms
} // namespace concord
