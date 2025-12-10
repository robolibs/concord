#include "concord/algorithms/intersection.hpp"

#include "concord/algorithms/spatial_algorithms.hpp"
#include <cmath>

namespace concord {
    namespace algorithms {
        namespace intersection {

            std::optional<Point> line_line(const Line &line1, const Line &line2) {
                Point result;
                if (spatial::lineIntersection(line1, line2, result)) {
                    return result;
                }
                return std::nullopt;
            }

            std::optional<Point> ray_plane(const Point &ray_origin, const Point &ray_direction,
                                           const Point &plane_point, const Point &plane_normal) {
                double denom = plane_normal.x * ray_direction.x + plane_normal.y * ray_direction.y +
                               plane_normal.z * ray_direction.z;
                if (std::abs(denom) < 1e-10) {
                    return std::nullopt;
                }

                double t =
                    (plane_normal.x * (plane_point.x - ray_origin.x) + plane_normal.y * (plane_point.y - ray_origin.y) +
                     plane_normal.z * (plane_point.z - ray_origin.z)) /
                    denom;

                if (t < 0.0) {
                    return std::nullopt;
                }

                Point hit;
                hit.x = ray_origin.x + t * ray_direction.x;
                hit.y = ray_origin.y + t * ray_direction.y;
                hit.z = ray_origin.z + t * ray_direction.z;
                return hit;
            }

            bool sphere_ray(const Point &sphere_center, double sphere_radius, const Point &ray_origin,
                            const Point &ray_direction) {
                Point m;
                m.x = ray_origin.x - sphere_center.x;
                m.y = ray_origin.y - sphere_center.y;
                m.z = ray_origin.z - sphere_center.z;

                double b = m.x * ray_direction.x + m.y * ray_direction.y + m.z * ray_direction.z;
                double c = m.x * m.x + m.y * m.y + m.z * m.z - sphere_radius * sphere_radius;

                if (c > 0.0 && b > 0.0) {
                    return false;
                }

                double discr = b * b - c;
                if (discr < 0.0) {
                    return false;
                }

                return true;
            }

        } // namespace intersection
    } // namespace algorithms
} // namespace concord
