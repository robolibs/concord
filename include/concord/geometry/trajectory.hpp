#pragma once

#include "../types/state.hpp"
#include <cstddef>
#include <vector>

namespace concord {

    class Trajectory {
      public:
        Trajectory() = default;
        explicit Trajectory(const std::vector<State> &states);

        void add(const State &s);
        void clear() noexcept;

        std::size_t size() const noexcept;
        bool empty() const noexcept;

        State &operator[](std::size_t idx);
        const State &operator[](std::size_t idx) const;

        std::vector<State>::iterator begin() noexcept;
        std::vector<State>::iterator end() noexcept;
        std::vector<State>::const_iterator begin() const noexcept;
        std::vector<State>::const_iterator end() const noexcept;

        const std::vector<State> &get() const noexcept;

      private:
        std::vector<State> states_;
    };

} // namespace concord
