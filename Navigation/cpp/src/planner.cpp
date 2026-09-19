#include "pyrobo/planner.h"
#include <queue>
#include <algorithm>

namespace pyrobo {

    namespace {
        bool segment_is_free(const Map& map, double x0, double y0, double x1, double y1) {
            const double dx = x1 - x0;
            const double dy = y1 - y0;
            const double distance = std::hypot(dx, dy);

            // 采样步长必须 <= resolution，否则可能"跨过"一整格障碍而不自知
            const int samples = static_cast<int>(std::ceil(distance / map.resolution()));
            for (int i = 0; i <= samples; ++i) {
                const double t = (samples == 0) ? 0.0
                                                : static_cast<double>(i) / samples;
                if (!map.is_free(x0 + dx * t, y0 + dy * t)) {
                    return false;
                }
            }

            return true;
        }

        Path simplify_path(const Path& path, const Map& map) {
            if (path.size() <= 2) {
                return path;
            }

            Path simplified;
            simplified.reserve(path.size());
            simplified.push_back(path.front());

            std::size_t anchor = 0;                    // 当前锚点（已确定保留）
            while (anchor + 1 < path.size()) {
                // 从最远处往回试：能拉直拉多远就拉多远
                std::size_t chosen = anchor + 1;
                for (std::size_t candidate = path.size() - 1; candidate > anchor + 1;
                    --candidate) {
                    if (segment_is_free(map, path[anchor].x, path[anchor].y,
                                        path[candidate].x, path[candidate].y)) {
                        chosen = candidate;
                        break;
                    }
                }
                simplified.push_back(path[chosen]);
                anchor = chosen;
            }
            return simplified;
        }
    } // namespace 
    

Path PathPlanner::plan(
    const Pose& robot_pose, const Pose& goal_pose, const Map& map
) {
    // Question 1: implement route planning here.
    // 这里使用 A*8邻域 + 欧氏启发函数 + 视线简化 的算法
    const auto data = map.data();
    const auto [height, width] = map.shape();
    const auto resolution = map.resolution();
    const auto start = map.world_to_grid(robot_pose.x, robot_pose.y);
    const auto goal = map.world_to_grid(goal_pose.x, goal_pose.y);

    const auto count = height * width;

    const auto flatten = [width](int column, int row) {
        return static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
               static_cast<std::size_t>(column);
    };

    const auto heuristic = [&goal, resolution](int column, int row) {
        const double dx = static_cast<double>(column - goal.column) * resolution;
        const double dy = static_cast<double>(row - goal.row) * resolution;
        return 0.5 * std::hypot(dx, dy);
    };

    const auto inside = [height, width](int column, int row) {
        return row >= 0 && row < height && column >= 0 && column < width;
    };

    const auto src = flatten(start.column, start.row);
    const auto tar = flatten(goal.column, goal.row);

    const auto inf = std::numeric_limits<double>::infinity();
    std::vector<double> g_score(static_cast<std::size_t>(count), inf);
    std::vector<int> parent(static_cast<std::size_t>(count), -1);
    std::vector<std::uint8_t> closed(static_cast<std::size_t>(count), 0);

    using Node = std::pair<double, std::size_t>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    
    g_score[src] = 0.0;
    open.emplace(heuristic(start.column, start.row), src);

    constexpr int dirs[8][2] = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1},
    };
    const auto cost_st = 1.0;
    const auto cost_cr = sqrt(2.0);
    bool reached = false;
    while(!open.empty()){
        const Node curr = open.top();
        open.pop();
        const std::size_t cell = curr.second;
        if(closed[cell] != 0){
            continue;
        }

        closed[cell] = 1;

        if (cell == tar){
            reached = true;
            break;
        }

        const int column = static_cast<int>(cell % static_cast<std::size_t>(width));
        const int row = static_cast<int>(cell / static_cast<std::size_t>(width));

        for (const auto& dir : dirs) {
            const int dr = dir[0];
            const int dc = dir[1];
            const int next_column = column + dc;
            const int next_row = row + dr;
            if (!inside(next_column, next_row) ||
                !map.is_free_cell(next_column, next_row)) {
                continue;
            }
            // 对角移动必须两个相邻格都可通行，否则会从障碍的拐角"切"过去
            if (dr != 0 && dc != 0 &&
                (!map.is_free_cell(column + dc, row) ||
                 !map.is_free_cell(column, row + dr))) {
                continue;
            }

            const std::size_t next = flatten(next_column, next_row);
            const double step = (dr != 0 && dc != 0) ? cost_st: cost_cr;
            const double tentative = g_score[cell] + step;
            if (tentative >= g_score[next]) {
                continue;
            }

            g_score[next] = tentative;
            parent[next] = static_cast<int>(cell);
            open.emplace(tentative + heuristic(next_column, next_row), next);
        }
    }

    if (!reached) {
        return {};
    }
    Path path;
    for (int node = static_cast<int>(tar); node >= 0;
         node = parent[static_cast<std::size_t>(node)]) {
        const int column = node % width;
        const int row = node / width;
        path.push_back(map.grid_to_world(column, row));   // 取格子中心
    }
    std::reverse(path.begin(), path.end());               // 变成 起点 -> 终点

    // 让路径真正从车当前位置出发，而不是从起点格中心出发
    if (std::hypot(path.front().x - robot_pose.x, path.front().y - robot_pose.y) >
        0.5 * resolution) {
        path.insert(path.begin(), Point{robot_pose.x, robot_pose.y});
    }

    return simplify_path(path, map);
}

std::pair<float, float> ControlPlanner::control_plan(
    const Pose& robot_pose, const Pose& goal_pose, const Map& map,
    const Feedback& fdb
) {
    // Question 3: implement control planning here.
    return {0.0F, 0.0F};
}

}  // namespace pyrobo
