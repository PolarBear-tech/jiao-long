#include <memory>
#include <stdexcept>
#include <utility>

// 调试所用
#include <iostream>

#include "pyrobo/interface.hpp"
#include "pyrobo/planner.h"

namespace {

class ContestantNavigationContext final : public pyrobo::NavigationContext {
   public:
    ContestantNavigationContext(pyrobo::Map planning_map, double robot_radius)
        : planning_map_(std::move(planning_map)), robot_radius_(robot_radius) {
    }

    const pyrobo::Map& planning_map() const noexcept {
        return planning_map_;
    }

    double robot_radius() const noexcept {
        return robot_radius_;
    }

    pyrobo::PathPlanner& path_planner() noexcept {
        return path_planner_;
    }

    pyrobo::ControlPlanner& control_planner() noexcept {
        return control_planner_;
    }

   private:
    pyrobo::Map planning_map_;
    double robot_radius_ = 0.0;
    pyrobo::PathPlanner path_planner_;
    pyrobo::ControlPlanner control_planner_;
};

}  // namespace

namespace pyrobo {

std::unique_ptr<NavigationContext> nav_init(Simulator& sim) {
    const double robot_radius = sim.get_robot().radius;
    // Question 2: do robot-radius preprocessing once during initialization.
    // This is where map inflation, clearance data or a collision model belongs.
    Map map = sim.map();

    // 先膨胀一下地图
    const auto resolution = map.resolution();
    const auto inflation_num = ceil(robot_radius / resolution);

    const auto data = map.data();
    const auto [height, width] = map.shape();
    auto inflation = data;

    for(int y = 0; y < height; y++){
        for (int x = 0; x < width; x++){
            if (data[y * width + x] == 0){
                for (int dy= -inflation_num; dy <= inflation_num; dy++){
                    for (int dx = -inflation_num; dx <= inflation_num; dx++){
                            int y0 = y + dy;
                            int x0 = x + dx;
                            if (y0 >= 0 && y0 < height &&
                                x0 >= 0 && x0 < width  &&
                                (dx * dx + dy * dy <= inflation_num * inflation_num)){
                                inflation[y0 * width + x0] = 0;
                            }
                    }
                }
            }
        }
    }
    map = pyrobo::Map(
        height, width, resolution, map.origin(), inflation
    );

    sim.set_planning_map(map);

    return std::make_unique<ContestantNavigationContext>(map, robot_radius);
}

void nav_run(Simulator& sim, NavigationContext& context) {
    auto* navigation_context =
        dynamic_cast<ContestantNavigationContext*>(&context);
    if (navigation_context == nullptr) {
        throw std::runtime_error("invalid navigation context");
    }

    const auto goal = sim.get_goal();
    if (!goal.has_value()) {
        sim.set_display_path({});
        sim.set_control(0.0, 0.0);
        return;
    }

    const Pose pose = sim.get_pose();
    const Feedback feedback = sim.get_feedback();
    const Map& planning_map = navigation_context->planning_map();

    // Question 4: use sim.get_lidar() to find and pass through the
    // unknown opening before planning the final route.
    const auto lidar_data = sim.get_lidar();

    const Path path =
        navigation_context->path_planner().plan(pose, *goal, planning_map);
    sim.set_display_path(path);

    const auto control = navigation_context->control_planner()
                             .control_plan(pose, *goal, planning_map, feedback);
    sim.set_control(control.first, control.second);
}

}  // namespace pyrobo
