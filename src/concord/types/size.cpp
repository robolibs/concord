#include "concord/types/size.hpp"
#include <algorithm>
#include <cmath>

namespace concord {

    Size::Size(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Size::Size(double s) : x(s), y(s), z(s) {}

    bool Size::is_set() const { return x != 0.0 || y != 0.0 || z != 0.0; }

    Size Size::operator+(const Size &other) const { return Size{x + other.x, y + other.y, z + other.z}; }

    Size Size::operator-(const Size &other) const { return Size{x - other.x, y - other.y, z - other.z}; }

    Size Size::operator*(double scale) const { return Size{x * scale, y * scale, z * scale}; }

    Size Size::operator/(double scale) const { return Size{x / scale, y / scale, z / scale}; }

    Size Size::operator*(const Size &other) const { return Size{x * other.x, y * other.y, z * other.z}; }

    bool Size::operator==(const Size &other) const { return x == other.x && y == other.y && z == other.z; }

    bool Size::operator!=(const Size &other) const { return !(*this == other); }

    double Size::volume() const { return x * y * z; }

    double Size::area_xy() const { return x * y; }

    double Size::area_xz() const { return x * z; }

    double Size::area_yz() const { return y * z; }

    double Size::diagonal() const { return std::sqrt(x * x + y * y + z * z); }

    double Size::diagonal_2d() const { return std::sqrt(x * x + y * y); }

    Size Size::abs() const { return Size{std::abs(x), std::abs(y), std::abs(z)}; }

    Size Size::max(const Size &other) const {
        return Size{std::max(x, other.x), std::max(y, other.y), std::max(z, other.z)};
    }

    Size Size::min(const Size &other) const {
        return Size{std::min(x, other.x), std::min(y, other.y), std::min(z, other.z)};
    }

} // namespace concord
