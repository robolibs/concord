#include "concord/geometry/primitives/square.hpp"
#include <cmath>

namespace concord {

    Square::Square(const Point &c, double s) : center(c), side(s) {}

    double Square::area() const noexcept { return side * side; }

    double Square::perimeter() const noexcept { return 4 * side; }

    double Square::diagonal() const noexcept { return side * std::sqrt(2.0); }

    bool Square::contains(const Point &p) const noexcept {
        return std::abs(p.x - center.x) <= side / 2.0 && std::abs(p.y - center.y) <= side / 2.0;
    }

    const Point &Square::getCenter() const noexcept { return center; }

    double Square::getSide() const noexcept { return side; }

} // namespace concord
