#include "pyrobo/planner.h"

namespace pyrobo {

Path PathPlanner::plan(
    const Pose& robot_pose, const Pose& goal_pose, const Map& map
) {
    // Question 1: implement route planning here.
    return Path{{robot_pose.x, robot_pose.y}, {goal_pose.x, goal_pose.y}};
}

std::pair<float, float> ControlPlanner::control_plan(
    const Pose& robot_pose, const Pose& goal_pose, const Map& map,
    const Feedback& fdb
) {
    // Question 3: implement control planning here.
    return {0.0F, 0.0F};
}

}  // namespace pyrobo
