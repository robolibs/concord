#pragma once

#include "../types/pose.hpp"
#include <cstddef>
#include <vector>

namespace concord {

    class Path {
      public:
        Path() = default;
        explicit Path(const std::vector<Pose> &poses);

        void add(const Pose &p);
        void clear() noexcept;

        std::size_t size() const noexcept;
        bool empty() const noexcept;

        Pose &operator[](std::size_t idx);
        const Pose &operator[](std::size_t idx) const;

        std::vector<Pose>::iterator begin() noexcept;
        std::vector<Pose>::iterator end() noexcept;
        std::vector<Pose>::const_iterator begin() const noexcept;
        std::vector<Pose>::const_iterator end() const noexcept;

        const std::vector<Pose> &get() const noexcept;

      private:
        std::vector<Pose> poses_;
    };

} // namespace concord
