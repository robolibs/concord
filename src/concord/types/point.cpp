#include "concord/types/point.hpp"
#include <cmath>

namespace concord {

    Point::Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Point::Point(double x_, double y_) : x(x_), y(y_), z(0.0) {}

    bool Point::is_set() const { return x != 0.0 && y != 0.0; }

    Point Point::operator+(const Point &other) const { return Point{x + other.x, y + other.y, z + other.z}; }

    Point Point::operator-(const Point &other) const { return Point{x - other.x, y - other.y, z - other.z}; }

    Point Point::operator*(double scale) const { return Point{x * scale, y * scale, z * scale}; }

    Point Point::operator/(double scale) const { return Point{x / scale, y / scale, z / scale}; }

    Point &Point::operator+=(const Point &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Point &Point::operator-=(const Point &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Point &Point::operator*=(double scale) {
        x *= scale;
        y *= scale;
        z *= scale;
        return *this;
    }

    Point &Point::operator/=(double scale) {
        x /= scale;
        y /= scale;
        z /= scale;
        return *this;
    }

    bool Point::operator==(const Point &other) const { return x == other.x && y == other.y && z == other.z; }

    bool Point::operator!=(const Point &other) const { return !(*this == other); }

    double Point::magnitude() const { return std::sqrt(x * x + y * y + z * z); }

    double Point::distance_to(const Point &other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y) + (z - other.z) * (z - other.z));
    }

    double Point::distance_to_2d(const Point &other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }

    Vec3d Point::to_vec3() const { return Vec3d{x, y, z}; }

    Point Point::from_vec3(const Vec3d &v) { return Point{v[0], v[1], v[2]}; }

} // namespace concord
