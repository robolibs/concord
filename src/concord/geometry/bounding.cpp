#include "concord/geometry/bounding.hpp"
#include <algorithm>
#include <cmath>

namespace concord {

    // AABB Implementation
    AABB::AABB(const Point &min_p, const Point &max_p) : min_point(min_p), max_point(max_p) {}

    void AABB::expand(const Point &point) {
        min_point.x = std::min(min_point.x, point.x);
        min_point.y = std::min(min_point.y, point.y);
        min_point.z = std::min(min_point.z, point.z);

        max_point.x = std::max(max_point.x, point.x);
        max_point.y = std::max(max_point.y, point.y);
        max_point.z = std::max(max_point.z, point.z);
    }

    bool AABB::contains(const Point &point) const {
        return (point.x >= min_point.x && point.x <= max_point.x && point.y >= min_point.y && point.y <= max_point.y &&
                point.z >= min_point.z && point.z <= max_point.z);
    }

    bool AABB::intersects(const AABB &other) const {
        return (min_point.x <= other.max_point.x && max_point.x >= other.min_point.x &&
                min_point.y <= other.max_point.y && max_point.y >= other.min_point.y &&
                min_point.z <= other.max_point.z && max_point.z >= other.min_point.z);
    }

    Point AABB::center() const {
        Point center_pt;
        center_pt.x = (min_point.x + max_point.x) * 0.5;
        center_pt.y = (min_point.y + max_point.y) * 0.5;
        center_pt.z = (min_point.z + max_point.z) * 0.5;
        return center_pt;
    }

    Size AABB::size() const {
        return Size{max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z};
    }

    double AABB::volume() const {
        auto s = size();
        return s.x * s.y * s.z;
    }

    double AABB::surface_area() const {
        auto s = size();
        return 2.0 * (s.x * s.y + s.y * s.z + s.z * s.x);
    }

    AABB AABB::union_with(const AABB &other) const {
        AABB result;
        result.min_point.x = std::min(min_point.x, other.min_point.x);
        result.min_point.y = std::min(min_point.y, other.min_point.y);
        result.min_point.z = std::min(min_point.z, other.min_point.z);
        result.max_point.x = std::max(max_point.x, other.max_point.x);
        result.max_point.y = std::max(max_point.y, other.max_point.y);
        result.max_point.z = std::max(max_point.z, other.max_point.z);
        return result;
    }

    double AABB::area() const { return volume(); }

    double AABB::distance_to_point(const Point &point) const {
        Point closest_point;
        closest_point.x = std::max(min_point.x, std::min(point.x, max_point.x));
        closest_point.y = std::max(min_point.y, std::min(point.y, max_point.y));
        closest_point.z = std::max(min_point.z, std::min(point.z, max_point.z));
        return point.distance_to(closest_point);
    }

    std::array<Point, 8> AABB::corners() const {
        return {Point{min_point.x, min_point.y, min_point.z}, Point{max_point.x, min_point.y, min_point.z},
                Point{max_point.x, max_point.y, min_point.z}, Point{min_point.x, max_point.y, min_point.z},
                Point{min_point.x, min_point.y, max_point.z}, Point{max_point.x, min_point.y, max_point.z},
                Point{max_point.x, max_point.y, max_point.z}, Point{min_point.x, max_point.y, max_point.z}};
    }

    bool AABB::operator==(const AABB &other) const {
        return min_point == other.min_point && max_point == other.max_point;
    }

    bool AABB::operator!=(const AABB &other) const { return !(*this == other); }

    // OBB Implementation
    OBB::OBB(const Point &c, const Size &he, const Euler &orient) : center(c), half_extents(he), orientation(orient) {}

    bool OBB::contains(const Point &point) const {
        double dx = point.x - center.x;
        double dy = point.y - center.y;
        double dz = point.z - center.z;

        double cos_yaw = std::cos(-orientation.yaw);
        double sin_yaw = std::sin(-orientation.yaw);

        double local_x = dx * cos_yaw - dy * sin_yaw;
        double local_y = dx * sin_yaw + dy * cos_yaw;
        double local_z = dz;

        return (std::abs(local_x) <= half_extents.x && std::abs(local_y) <= half_extents.y &&
                std::abs(local_z) <= half_extents.z);
    }

    std::array<Point, 8> OBB::corners() const {
        std::array<Point, 8> corners;

        std::array<std::array<double, 3>, 8> local_corners = {{{{-half_extents.x, -half_extents.y, -half_extents.z}},
                                                               {{+half_extents.x, -half_extents.y, -half_extents.z}},
                                                               {{+half_extents.x, +half_extents.y, -half_extents.z}},
                                                               {{-half_extents.x, +half_extents.y, -half_extents.z}},
                                                               {{-half_extents.x, -half_extents.y, +half_extents.z}},
                                                               {{+half_extents.x, -half_extents.y, +half_extents.z}},
                                                               {{+half_extents.x, +half_extents.y, +half_extents.z}},
                                                               {{-half_extents.x, +half_extents.y, +half_extents.z}}}};

        double cos_yaw = std::cos(orientation.yaw);
        double sin_yaw = std::sin(orientation.yaw);

        for (size_t i = 0; i < 8; ++i) {
            double lx = local_corners[i][0];
            double ly = local_corners[i][1];
            double lz = local_corners[i][2];

            double world_x = center.x + (lx * cos_yaw - ly * sin_yaw);
            double world_y = center.y + (lx * sin_yaw + ly * cos_yaw);
            double world_z = center.z + lz;

            corners[i] = Point{world_x, world_y, world_z};
        }

        return corners;
    }

    bool OBB::operator==(const OBB &other) const {
        return center == other.center && half_extents == other.half_extents && orientation == other.orientation;
    }

    bool OBB::operator!=(const OBB &other) const { return !(*this == other); }

    // BoundingSphere Implementation
    BoundingSphere::BoundingSphere(const Point &c, double r) : center(c), radius(r) {}

    bool BoundingSphere::contains(const Point &point) const {
        double dx = point.x - center.x;
        double dy = point.y - center.y;
        double dz = point.z - center.z;
        double dist_sq = dx * dx + dy * dy + dz * dz;
        return dist_sq <= radius * radius;
    }

    bool BoundingSphere::intersects(const BoundingSphere &other) const {
        double dx = other.center.x - center.x;
        double dy = other.center.y - center.y;
        double dz = other.center.z - center.z;
        double dist_sq = dx * dx + dy * dy + dz * dz;
        double sum_radii = radius + other.radius;
        return dist_sq <= sum_radii * sum_radii;
    }

    double BoundingSphere::volume() const { return (4.0 / 3.0) * M_PI * radius * radius * radius; }

    double BoundingSphere::surface_area() const { return 4.0 * M_PI * radius * radius; }

    bool BoundingSphere::operator==(const BoundingSphere &other) const {
        return center == other.center && radius == other.radius;
    }

    bool BoundingSphere::operator!=(const BoundingSphere &other) const { return !(*this == other); }

} // namespace concord
