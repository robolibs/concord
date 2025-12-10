#pragma once

#include "../types/point.hpp"
#include <cstddef>
#include <vector>

namespace concord {

    class Path {
      public:
        Path() = default;
        explicit Path(const std::vector<Point> &pts);

        void addPoint(const Point &p);
        void clear() noexcept;

        std::size_t size() const noexcept;
        bool empty() const noexcept;

        Point &operator[](std::size_t idx);
        const Point &operator[](std::size_t idx) const;

        auto begin() noexcept;
        auto end() noexcept;
        auto begin() const noexcept;
        auto end() const noexcept;

        const std::vector<Point> &getPoints() const noexcept;

      private:
        std::vector<Point> points;
    };

} // namespace concord
