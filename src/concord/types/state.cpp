#include "concord/types/state.hpp"

namespace concord {

    State::State(const Pose &p, double linear_velocity, double angular_velocity)
        : pose(p), lin_vel(linear_velocity), ang_vel(angular_velocity) {}

    bool State::is_set() const { return pose.is_set(); }

    bool State::operator==(const State &other) const {
        return pose == other.pose && lin_vel == other.lin_vel && ang_vel == other.ang_vel;
    }

    bool State::operator!=(const State &other) const { return !(*this == other); }

} // namespace concord
