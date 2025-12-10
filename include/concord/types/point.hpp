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

        Point(double x_, double y_, double z_);
        Point(double x_, double y_);
        Point() = default;

        // Forward declaration for conversion method - implementation in crs.hpp
        WGS toWGS(const Datum &datum) const;

        bool is_set() const;

        // Mathematical operations
        Point operator+(const Point &other) const;
        Point operator-(const Point &other) const;
        Point operator*(double scale) const;
        Point operator/(double scale) const;

        Point &operator+=(const Point &other);
        Point &operator-=(const Point &other);
        Point &operator*=(double scale);
        Point &operator/=(double scale);

        bool operator==(const Point &other) const;
        bool operator!=(const Point &other) const;

        // Distance and magnitude operations
        double magnitude() const;
        double distance_to(const Point &other) const;
        double distance_to_2d(const Point &other) const;

        // Conversion to Vec3d
        Vec3d to_vec3() const;
        static Point from_vec3(const Vec3d &v);
    };

} // namespace concord
