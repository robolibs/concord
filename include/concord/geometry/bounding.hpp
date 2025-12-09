#pragma once

#include "../geographic/crs/datum.hpp"
#include "../types/euler.hpp"
#include "../types/point.hpp"
#include "../types/size.hpp"
#include <algorithm>
#include <array>
#include <limits>

namespace concord {

    // Axis-Aligned Bounding Box (AABB)
    struct AABB {
        Point min_point;
        Point max_point;

        AABB() = default;
        AABB(const Point &min_p, const Point &max_p) : min_point(min_p), max_point(max_p) {}

        // Create AABB from collection of points
        template <typename Container> static AABB fromPoints(const Container &points) {
            if (points.empty()) {
                return AABB{};
            }

            auto it = points.begin();
            AABB result;
            result.min_point = *it;
            result.max_point = *it;

            for (++it; it != points.end(); ++it) {
                result.expand(*it);
            }
            return result;
        }

        inline void expand(const Point &point) {
            min_point.x = std::min(min_point.x, point.x);
            min_point.y = std::min(min_point.y, point.y);
            min_point.z = std::min(min_point.z, point.z);

            max_point.x = std::max(max_point.x, point.x);
            max_point.y = std::max(max_point.y, point.y);
            max_point.z = std::max(max_point.z, point.z);
        }

        inline bool contains(const Point &point) const {
            return (point.x >= min_point.x && point.x <= max_point.x && point.y >= min_point.y &&
                    point.y <= max_point.y && point.z >= min_point.z && point.z <= max_point.z);
        }

        inline bool intersects(const AABB &other) const {
            return (min_point.x <= other.max_point.x && max_point.x >= other.min_point.x &&
                    min_point.y <= other.max_point.y && max_point.y >= other.min_point.y &&
                    min_point.z <= other.max_point.z && max_point.z >= other.min_point.z);
        }

        inline Point center() const {
            Point center_pt;
            center_pt.x = (min_point.x + max_point.x) * 0.5;
            center_pt.y = (min_point.y + max_point.y) * 0.5;
            center_pt.z = (min_point.z + max_point.z) * 0.5;
            return center_pt;
        }

        inline Size size() const {
            return Size{max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z};
        }

        inline double volume() const {
            auto s = size();
            return s.x * s.y * s.z;
        }

        inline double surface_area() const {
            auto s = size();
            return 2.0 * (s.x * s.y + s.y * s.z + s.z * s.x);
        }

        // Union with another AABB - returns a new AABB that contains both
        inline AABB union_with(const AABB &other) const {
            AABB result;
            result.min_point.x = std::min(min_point.x, other.min_point.x);
            result.min_point.y = std::min(min_point.y, other.min_point.y);
            result.min_point.z = std::min(min_point.z, other.min_point.z);
            result.max_point.x = std::max(max_point.x, other.max_point.x);
            result.max_point.y = std::max(max_point.y, other.max_point.y);
            result.max_point.z = std::max(max_point.z, other.max_point.z);
            return result;
        }

        // Area for 2D or volume for 3D
        inline double area() const { return volume(); }

        // Distance from AABB to a point
        inline double distance_to_point(const Point &point) const {
            Point closest_point;
            closest_point.x = std::max(min_point.x, std::min(point.x, max_point.x));
            closest_point.y = std::max(min_point.y, std::min(point.y, max_point.y));
            closest_point.z = std::max(min_point.z, std::min(point.z, max_point.z));
            return point.distance_to(closest_point);
        }

        inline std::array<Point, 8> corners() const {
            return {Point{min_point.x, min_point.y, min_point.z}, Point{max_point.x, min_point.y, min_point.z},
                    Point{max_point.x, max_point.y, min_point.z}, Point{min_point.x, max_point.y, min_point.z},
                    Point{min_point.x, min_point.y, max_point.z}, Point{max_point.x, min_point.y, max_point.z},
                    Point{max_point.x, max_point.y, max_point.z}, Point{min_point.x, max_point.y, max_point.z}};
        }

        inline bool operator==(const AABB &other) const {
            return min_point == other.min_point && max_point == other.max_point;
        }

        inline bool operator!=(const AABB &other) const { return !(*this == other); }
    };

    // Oriented Bounding Box (OBB) - more precise than AABB
    struct OBB {
        Point center;
        Size half_extents;
        Euler orientation;

        OBB() = default;
        OBB(const Point &c, const Size &he, const Euler &orient) : center(c), half_extents(he), orientation(orient) {}

        // Create OBB from points using PCA or convex hull + rotating calipers
        template <typename Container> static OBB fromPoints(const Container &points, const Datum & /* datum */ = {}) {
            if (points.empty()) {
                return OBB{};
            }

            // For now, use AABB as approximation
            // TODO: Implement proper PCA-based OBB fitting
            auto aabb = AABB::fromPoints(points);
            return OBB{aabb.center(), Size{aabb.size().x * 0.5, aabb.size().y * 0.5, aabb.size().z * 0.5}, Euler{}};
        }

        inline bool contains(const Point &point) const {
            // Transform point to OBB local space
            double dx = point.x - center.x;
            double dy = point.y - center.y;
            double dz = point.z - center.z;

            // Apply inverse rotation
            double cos_yaw = std::cos(-orientation.yaw);
            double sin_yaw = std::sin(-orientation.yaw);
            // double cos_pitch = std::cos(-orientation.pitch);  // Currently unused
            // double sin_pitch = std::sin(-orientation.pitch);  // Currently unused
            // double cos_roll = std::cos(-orientation.roll);    // Currently unused
            // double sin_roll = std::sin(-orientation.roll);    // Currently unused

            // Simplified rotation (yaw only for 2D case)
            double local_x = dx * cos_yaw - dy * sin_yaw;
            double local_y = dx * sin_yaw + dy * cos_yaw;
            double local_z = dz;

            return (std::abs(local_x) <= half_extents.x && std::abs(local_y) <= half_extents.y &&
                    std::abs(local_z) <= half_extents.z);
        }

        inline std::array<Point, 8> corners() const {
            std::array<Point, 8> corners;

            // Local corners relative to center
            std::array<std::array<double, 3>, 8> local_corners = {
                {{{-half_extents.x, -half_extents.y, -half_extents.z}},
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
                // Apply rotation and translation
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

        inline bool operator==(const OBB &other) const {
            return center == other.center && half_extents == other.half_extents && orientation == other.orientation;
        }

        inline bool operator!=(const OBB &other) const { return !(*this == other); }
    };

    // Sphere/Circle bounding volume
    struct BoundingSphere {
        Point center;
        double radius = 0.0;

        BoundingSphere() = default;
        BoundingSphere(const Point &c, double r) : center(c), radius(r) {}

        template <typename Container> static BoundingSphere fromPoints(const Container &points) {
            if (points.empty()) {
                return BoundingSphere{};
            }

            // Naive algorithm: use centroid and max distance
            Point centroid;
            double sum_x = 0, sum_y = 0, sum_z = 0;
            size_t count = 0;

            for (const auto &point : points) {
                sum_x += point.x;
                sum_y += point.y;
                sum_z += point.z;
                ++count;
            }

            centroid.x = sum_x / count;
            centroid.y = sum_y / count;
            centroid.z = sum_z / count;

            double max_dist = 0.0;
            for (const auto &point : points) {
                double dx = point.x - centroid.x;
                double dy = point.y - centroid.y;
                double dz = point.z - centroid.z;
                double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                max_dist = std::max(max_dist, dist);
            }

            return BoundingSphere{centroid, max_dist};
        }

        inline bool contains(const Point &point) const {
            double dx = point.x - center.x;
            double dy = point.y - center.y;
            double dz = point.z - center.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;
            return dist_sq <= radius * radius;
        }

        inline bool intersects(const BoundingSphere &other) const {
            double dx = other.center.x - center.x;
            double dy = other.center.y - center.y;
            double dz = other.center.z - center.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;
            double sum_radii = radius + other.radius;
            return dist_sq <= sum_radii * sum_radii;
        }

        inline double volume() const { return (4.0 / 3.0) * M_PI * radius * radius * radius; }

        inline double surface_area() const { return 4.0 * M_PI * radius * radius; }

        inline bool operator==(const BoundingSphere &other) const {
            return center == other.center && radius == other.radius;
        }

        inline bool operator!=(const BoundingSphere &other) const { return !(*this == other); }
    };

} // namespace concord
