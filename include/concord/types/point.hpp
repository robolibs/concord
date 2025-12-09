#pragma once

#include "../math.hpp"
#include <cmath>
#include <tuple>

namespace concord {

    struct WGS;   // forward declaration
    struct Datum; // forward declaration

    struct Point {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
        Point(double x_, double y_) : x(x_), y(y_), z(0.0) {}

        Point() = default;

        // Forward declaration for conversion method - implementation in crs.hpp
        inline WGS toWGS(const Datum &datum) const;

        inline bool is_set() const { return x != 0.0 && y != 0.0; }

        // Mathematical operations
        inline Point operator+(const Point &other) const { return Point{x + other.x, y + other.y, z + other.z}; }
        inline Point operator-(const Point &other) const { return Point{x - other.x, y - other.y, z - other.z}; }
        inline Point operator*(double scale) const { return Point{x * scale, y * scale, z * scale}; }
        inline Point operator/(double scale) const { return Point{x / scale, y / scale, z / scale}; }

        inline Point &operator+=(const Point &other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }
        inline Point &operator-=(const Point &other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }
        inline Point &operator*=(double scale) {
            x *= scale;
            y *= scale;
            z *= scale;
            return *this;
        }
        inline Point &operator/=(double scale) {
            x /= scale;
            y /= scale;
            z /= scale;
            return *this;
        }

        inline bool operator==(const Point &other) const { return x == other.x && y == other.y && z == other.z; }

        inline bool operator!=(const Point &other) const { return !(*this == other); }

        // Distance and magnitude operations
        inline double magnitude() const { return std::sqrt(x * x + y * y + z * z); }
        inline double distance_to(const Point &other) const {
            return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y) +
                             (z - other.z) * (z - other.z));
        }
        inline double distance_to_2d(const Point &other) const {
            return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
        }

        // Conversion to Vec3d
        inline Vec3d to_vec3() const { return Vec3d{x, y, z}; }
        inline static Point from_vec3(const Vec3d &v) { return Point{v[0], v[1], v[2]}; }
    };

} // namespace concord
