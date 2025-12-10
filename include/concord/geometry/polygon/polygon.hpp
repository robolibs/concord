#pragma once

#include "../../types/bound.hpp"
#include "../bounding.hpp"
#include "../primitives/line.hpp"
#include <cmath>
#include <cstddef>
#include <vector>

namespace concord {

    class Polygon {
      public:
        Polygon() = default;
        explicit Polygon(const std::vector<Point> &pts);

        void addPoint(const Point &p);

        std::size_t numVertices() const noexcept;
        bool isConnected() const noexcept;

        double perimeter() const noexcept;
        double area() const noexcept;
        bool contains(const Point &p) const noexcept;

        Polygon from_rectangle(const float width, const float height, Size inflate = Size(1.0, 1.0, 1.0)) const;
        Polygon from_vector(std::vector<Point> pts);
        Polygon from_rectangle(Size size, Size inflate = Size(1.0, 1.0, 1.0)) const;

        Bound get_obb() const;

        auto begin() noexcept;
        auto end() noexcept;
        auto begin() const noexcept;
        auto end() const noexcept;

        const std::vector<Point> &getPoints() const noexcept;

        AABB getAABB() const;

        bool operator==(const Polygon &other) const;
        bool operator!=(const Polygon &other) const;

      private:
        std::vector<Point> points;
    };

} // namespace concord
