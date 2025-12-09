#pragma once

#include "point.hpp"
#include "pose.hpp"
#include "size.hpp"
#include <cmath>
#include <vector>

namespace concord {

    struct Bound {
        Pose pose;
        Size size;

        Bound() = default;
        Bound(const Pose &p, const Size &s);
        bool is_set() const;

        // Geometric properties
        double volume() const;
        double area() const;
        Point center() const;

        // Containment testing
        bool contains(const Point &point) const;

        // Intersection testing
        bool intersects(const Bound &other) const;

        // Expand bound to include point
        void expand_to_include(const Point &point);

        std::vector<Point> get_corners() const;

        bool operator==(const Bound &other) const;
        bool operator!=(const Bound &other) const;
    };

} // namespace concord
