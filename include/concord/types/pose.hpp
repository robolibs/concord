#pragma once

#include "../math.hpp"
#include "euler.hpp"
#include "point.hpp"
#include "quaternion.hpp"
#include "size.hpp"
#include <array>
#include <cmath>
#include <vector>

namespace concord {

    struct Pose {
        Point point;
        Euler angle;

        Pose() = default;
        Pose(const Point &p, const Euler &a);
        Pose(float x, float y, float yaw);
        explicit Pose(const Point &p, const Quaternion &q) noexcept;
        bool is_set() const;

        // Transformation operations
        Point transform_point(const Point &local_point) const;
        Point inverse_transform_point(const Point &world_point) const;

        // Pose composition
        Pose operator*(const Pose &other) const;

        // Inverse pose
        Pose inverse() const;

        bool operator==(const Pose &other) const;
        bool operator!=(const Pose &other) const;

        std::vector<Point> get_corners(Size size) const;
    };

} // namespace concord
