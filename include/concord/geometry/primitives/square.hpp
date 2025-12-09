#pragma once

#include "../../types/point.hpp"
#include <cmath>

namespace concord {

    class Square {
      public:
        Square() = default;
        Square(const Point &c, double s);

        double area() const noexcept;
        double perimeter() const noexcept;
        double diagonal() const noexcept;

        bool contains(const Point &p) const noexcept;

        const Point &getCenter() const noexcept;
        double getSide() const noexcept;

      private:
        Point center;
        double side;
    };

} // namespace concord
