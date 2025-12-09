#pragma once

#include "../types/point.hpp"
#include <cstddef>
#include <vector>

namespace concord {

    class Path {
      public:
        Path() = default;
        explicit Path(const std::vector<Point> &pts) : points(pts) {}

        void addPoint(const Point &p) { points.emplace_back(p); }
        void clear() noexcept { points.clear(); }

        std::size_t size() const noexcept { return points.size(); }
        bool empty() const noexcept { return points.empty(); }

        Point &operator[](std::size_t idx) { return points.at(idx); }
        const Point &operator[](std::size_t idx) const { return points.at(idx); }

        auto begin() noexcept { return points.begin(); }
        auto end() noexcept { return points.end(); }
        auto begin() const noexcept { return points.begin(); }
        auto end() const noexcept { return points.end(); }

        const std::vector<Point> &getPoints() const noexcept { return points; }

      private:
        std::vector<Point> points;
    };

} // namespace concord
