#pragma once

#include "pose.hpp"

namespace concord {

    struct State {
        Pose pose;
        double lin_vel = 0.0;
        double ang_vel = 0.0;

        State() = default;
        State(const Pose &p, double linear_velocity, double angular_velocity);

        bool is_set() const;

        bool operator==(const State &other) const;
        bool operator!=(const State &other) const;
    };

} // namespace concord
