#include "concord/geometry/primitives/line.hpp"
#include <cmath>

namespace concord {

    Line::Line(const Point &s, const Point &e) : start(s), end(e) {}

    double Line::length() const noexcept {
        const auto &a = start;
        const auto &b = end;
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double dz = b.z - a.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    const Point &Line::getStart() const noexcept { return start; }

    const Point &Line::getEnd() const noexcept { return end; }

    void Line::setStart(const Point &s) noexcept { start = s; }

    void Line::setEnd(const Point &e) noexcept { end = e; }

    bool Line::operator==(const Line &other) const { return start == other.start && end == other.end; }

    bool Line::operator!=(const Line &other) const { return !(*this == other); }

} // namespace concord
