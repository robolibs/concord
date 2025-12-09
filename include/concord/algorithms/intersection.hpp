#pragma once

#include "../geometry/primitives/line.hpp"
#include "../types/point.hpp"
#include <optional>

namespace concord {
    namespace algorithms {
        namespace intersection {

            // Line-line intersection
            std::optional<Point> line_line(const Line &line1, const Line &line2);

            // Ray-plane intersection
            std::optional<Point> ray_plane(const Point &ray_origin, const Point &ray_direction,
                                           const Point &plane_point, const Point &plane_normal);

            // Sphere-ray intersection
            bool sphere_ray(const Point &sphere_center, double sphere_radius, const Point &ray_origin,
                            const Point &ray_direction);

        } // namespace intersection
    } // namespace algorithms
} // namespace concord

