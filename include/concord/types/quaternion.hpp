#pragma once

#include "../math.hpp"
#include "euler.hpp"
#include <cmath>

namespace concord {

    struct Quaternion {
        double w = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        Quaternion() = default;
        Quaternion(double w_, double x_, double y_, double z_);
        explicit Quaternion(const Euler &e) noexcept;
        bool is_set() const;

        // Mathematical operations
        Quaternion operator+(const Quaternion &other) const;
        Quaternion operator-(const Quaternion &other) const;
        Quaternion operator*(const Quaternion &other) const;
        Quaternion operator*(double scale) const;

        bool operator==(const Quaternion &other) const;
        bool operator!=(const Quaternion &other) const;

        // Quaternion operations
        double norm() const;
        Quaternion normalized() const;
        Quaternion conjugate() const;
        Quaternion inverse() const;

        // Rotation operations
        Vec3d rotate(const Vec3d &v) const;

        // SLERP interpolation
        static Quaternion slerp(const Quaternion &q1, const Quaternion &q2, double t);
    };

} // namespace concord
