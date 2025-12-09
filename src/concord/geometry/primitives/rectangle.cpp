#include "concord/geometry/primitives/rectangle.hpp"
#include "concord/geometry/primitives/line.hpp"

namespace concord {

    Rectangle::Rectangle(const Point &tl, const Point &tr, const Point &bl, const Point &br)
        : top_left(tl), top_right(tr), bottom_left(bl), bottom_right(br) {}

    double Rectangle::area() const noexcept {
        double width = Line(top_left, top_right).length();
        double height = Line(top_left, bottom_left).length();
        return width * height;
    }

    double Rectangle::perimeter() const noexcept {
        double w = Line(top_left, top_right).length();
        double h = Line(top_left, bottom_left).length();
        return 2 * (w + h);
    }

    bool Rectangle::contains(const Point &p) const noexcept {
        return (p.x >= top_left.x && p.x <= top_right.x && p.y >= top_left.y && p.y <= bottom_left.y);
    }

    void Rectangle::from_pointvec(std::array<Point, 4> points) {
        top_left = points[0];
        top_right = points[1];
        bottom_left = points[2];
        bottom_right = points[3];
    }

    const Point &Rectangle::getTopLeft() const noexcept { return top_left; }

    const Point &Rectangle::getTopRight() const noexcept { return top_right; }

    const Point &Rectangle::getBottomLeft() const noexcept { return bottom_left; }

    const Point &Rectangle::getBottomRight() const noexcept { return bottom_right; }

    std::array<Point, 4> Rectangle::get_corners() const noexcept {
        return {top_left, top_right, bottom_right, bottom_left};
    }

    bool Rectangle::operator==(const Rectangle &other) const {
        return top_left == other.top_left && top_right == other.top_right && bottom_left == other.bottom_left &&
               bottom_right == other.bottom_right && orientation == other.orientation;
    }

    bool Rectangle::operator!=(const Rectangle &other) const { return !(*this == other); }

} // namespace concord
