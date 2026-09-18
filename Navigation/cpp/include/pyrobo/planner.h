#pragma once

#include <utility>

#include "pyrobo/interface.hpp"

namespace pyrobo {

class PathPlanner {
   public:
    Path plan(const Pose &robot_pose, const Pose &goal_pose, const Map &map);
};

class ControlPlanner {
   public:
    std::pair<float, float> control_plan(
        const Pose &robot_pose, const Pose &goal_pose, const Map &map,
        const Feedback &fdb  // vx, vy
    );
};

}  // namespace pyrobo
