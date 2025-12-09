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
        AABB(const Point &min_p, const Point &max_p);

        // Create AABB from collection of points (template must stay in header)
        template <typename Container> 
        static AABB fromPoints(const Container &points) {
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

        void expand(const Point &point);
        bool contains(const Point &point) const;
        bool intersects(const AABB &other) const;
        Point center() const;
        Size size() const;
        double volume() const;
        double surface_area() const;
        AABB union_with(const AABB &other) const;
        double area() const;
        double distance_to_point(const Point &point) const;
        std::array<Point, 8> corners() const;
        bool operator==(const AABB &other) const;
        bool operator!=(const AABB &other) const;
    };

    // Oriented Bounding Box (OBB) - more precise than AABB
    struct OBB {
        Point center;
        Size half_extents;
        Euler orientation;

        OBB() = default;
        OBB(const Point &c, const Size &he, const Euler &orient);

        // Create OBB from points (template must stay in header)
        template <typename Container> 
        static OBB fromPoints(const Container &points, const Datum & /* datum */ = {}) {
            if (points.empty()) {
                return OBB{};
            }

            // For now, use AABB as approximation
            // TODO: Implement proper PCA-based OBB fitting
            auto aabb = AABB::fromPoints(points);
            return OBB{aabb.center(), 
                      Size{aabb.size().x * 0.5, aabb.size().y * 0.5, aabb.size().z * 0.5}, 
                      Euler{}};
        }

        bool contains(const Point &point) const;
        std::array<Point, 8> corners() const;
        bool operator==(const OBB &other) const;
        bool operator!=(const OBB &other) const;
    };

    // Sphere/Circle bounding volume
    struct BoundingSphere {
        Point center;
        double radius = 0.0;

        BoundingSphere() = default;
        BoundingSphere(const Point &c, double r);

        // Create BoundingSphere from points (template must stay in header)
        template <typename Container> 
        static BoundingSphere fromPoints(const Container &points) {
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

        bool contains(const Point &point) const;
        bool intersects(const BoundingSphere &other) const;
        double volume() const;
        double surface_area() const;
        bool operator==(const BoundingSphere &other) const;
        bool operator!=(const BoundingSphere &other) const;
    };

} // namespace concord
