#pragma once

#include "../math.hpp"
#include <cmath>

namespace concord {

    struct Quaternion; // forward declaration

    struct Euler {
        double roll = 0.0;  // rotation about x-axis
        double pitch = 0.0; // rotation about y-axis
        double yaw = 0.0;   // rotation about z-axis

        Euler() = default;
        Euler(double roll_, double pitch_, double yaw_);
        explicit Euler(const Quaternion &q) noexcept;
        bool is_set() const;

        double yaw_cos() const;
        double yaw_sin() const;

        // Mathematical operations
        Euler operator+(const Euler &other) const;
        Euler operator-(const Euler &other) const;
        Euler operator*(double scale) const;

        bool operator==(const Euler &other) const;
        bool operator!=(const Euler &other) const;

        // Angle normalization
        Euler normalized() const;

        // Convert to rotation matrix (3x3)
        Mat3d to_rotation_matrix() const;

        // Rotate a vector
        Vec3d rotate(const Vec3d &v) const;
    };

} // namespace concord
