#include "concord/geometry/trajectory.hpp"

namespace concord {

    Trajectory::Trajectory(const std::vector<State> &states) : states_(states) {}

    void Trajectory::add(const State &s) { states_.emplace_back(s); }

    void Trajectory::clear() noexcept { states_.clear(); }

    std::size_t Trajectory::size() const noexcept { return states_.size(); }

    bool Trajectory::empty() const noexcept { return states_.empty(); }

    State &Trajectory::operator[](std::size_t idx) { return states_.at(idx); }

    const State &Trajectory::operator[](std::size_t idx) const { return states_.at(idx); }

    std::vector<State>::iterator Trajectory::begin() noexcept { return states_.begin(); }

    std::vector<State>::iterator Trajectory::end() noexcept { return states_.end(); }

    std::vector<State>::const_iterator Trajectory::begin() const noexcept { return states_.begin(); }

    std::vector<State>::const_iterator Trajectory::end() const noexcept { return states_.end(); }

    const std::vector<State> &Trajectory::get() const noexcept { return states_; }

} // namespace concord
