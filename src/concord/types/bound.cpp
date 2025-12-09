#include "concord/types/bound.hpp"
#include <cmath>

namespace concord {

    Bound::Bound(const Pose &p, const Size &s) : pose(p), size(s) {}

    bool Bound::is_set() const { return pose.is_set() && size.is_set(); }

    double Bound::volume() const { return size.volume(); }

    double Bound::area() const { return size.area_xy(); }

    Point Bound::center() const { return pose.point; }

    bool Bound::contains(const Point &point) const {
        Point local_point = pose.inverse_transform_point(point);
        double half_x = size.x * 0.5;
        double half_y = size.y * 0.5;
        double half_z = size.z * 0.5;
        return (std::abs(local_point.x) <= half_x && std::abs(local_point.y) <= half_y &&
                std::abs(local_point.z) <= half_z);
    }

    bool Bound::intersects(const Bound &other) const {
        // Simple OBB intersection test using separating axis theorem
        // This is a simplified version - full SAT would check all axes
        std::vector<Point> corners1 = get_corners();
        std::vector<Point> corners2 = other.get_corners();

        // Check if any corner of one bound is inside the other
        for (const auto &corner : corners1) {
            if (other.contains(corner))
                return true;
        }
        for (const auto &corner : corners2) {
            if (contains(corner))
                return true;
        }
        return false;
    }

    void Bound::expand_to_include(const Point &point) {
        if (!contains(point)) {
            // Transform point to local coordinates
            Point local_point = pose.inverse_transform_point(point);

            // Expand size if necessary
            double half_x = size.x * 0.5;
            double half_y = size.y * 0.5;
            double half_z = size.z * 0.5;

            if (std::abs(local_point.x) > half_x) {
                size.x = std::abs(local_point.x) * 2.0;
            }
            if (std::abs(local_point.y) > half_y) {
                size.y = std::abs(local_point.y) * 2.0;
            }
            if (std::abs(local_point.z) > half_z) {
                size.z = std::abs(local_point.z) * 2.0;
            }
        }
    }

    std::vector<Point> Bound::get_corners() const { return pose.get_corners(size); }

    bool Bound::operator==(const Bound &other) const { return pose == other.pose && size == other.size; }

    bool Bound::operator!=(const Bound &other) const { return !(*this == other); }

} // namespace concord
