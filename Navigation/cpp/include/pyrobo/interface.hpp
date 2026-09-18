#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace pyrobo {

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct GridIndex {
    int column = 0;
    int row = 0;
};

struct Shape {
    int height = 0;
    int width = 0;
};

struct RobotInfo {
    double radius = 0.0;
};

using Path = std::vector<Point>;
using Feedback = std::pair<double, double>;
using Control = std::pair<double, double>;

class Map {
   public:
    Map() = default;

    Map(int height, int width, double resolution, Point origin,
        std::vector<std::uint8_t> data)
        : height_(height),
          width_(width),
          resolution_(resolution),
          origin_(origin),
          data_(std::move(data)) {
        if (height_ <= 0 || width_ <= 0) {
            throw std::invalid_argument("map dimensions must be positive");
        }
        if (resolution_ <= 0.0) {
            throw std::invalid_argument("map resolution must be positive");
        }
        if (data_.size() != static_cast<std::size_t>(height_ * width_)) {
            throw std::invalid_argument("map data size does not match shape");
        }
    }

    const std::vector<std::uint8_t>& data() const noexcept {
        return data_;
    }

    Shape shape() const noexcept {
        return {height_, width_};
    }

    double resolution() const noexcept {
        return resolution_;
    }

    Point origin() const noexcept {
        return origin_;
    }

    Point size_meters() const noexcept {
        return {
            static_cast<double>(width_) * resolution_,
            static_cast<double>(height_) * resolution_,
        };
    }

    GridIndex world_to_grid(double x, double y) const noexcept {
        const int column =
            static_cast<int>(std::floor((x - origin_.x) / resolution_));
        const int from_bottom =
            static_cast<int>(std::floor((y - origin_.y) / resolution_));
        return {column, height_ - 1 - from_bottom};
    }

    Point grid_to_world(int column, int row) const {
        if (row < 0 || row >= height_ || column < 0 || column >= width_) {
            throw std::out_of_range("grid coordinate is outside the map");
        }
        return {
            origin_.x + (static_cast<double>(column) + 0.5) * resolution_,
            origin_.y +
                (static_cast<double>(height_ - row) - 0.5) * resolution_,
        };
    }

    bool is_free(double x, double y) const noexcept {
        const auto index = world_to_grid(x, y);
        return is_free_cell(index.column, index.row);
    }

    bool is_free_cell(int column, int row) const noexcept {
        return row >= 0 && row < height_ && column >= 0 && column < width_ &&
               data_[static_cast<std::size_t>(row * width_ + column)] != 0;
    }

   private:
    int height_ = 0;
    int width_ = 0;
    double resolution_ = 1.0;
    Point origin_{};
    std::vector<std::uint8_t> data_;
};

class Simulator {
   public:
    virtual ~Simulator() = default;

    virtual const Map& map() const = 0;

    virtual Pose get_pose(std::string_view robot_id = "robot") const = 0;
    virtual RobotInfo get_robot(std::string_view robot_id = "robot") const = 0;

    virtual std::optional<Pose> get_goal() const = 0;

    virtual Feedback get_feedback(std::string_view robot_id = "robot")
        const = 0;

    virtual void set_control(
        double first, double second, std::string_view robot_id = "robot"
    ) = 0;
    virtual Control get_control(std::string_view robot_id = "robot") const = 0;

    virtual std::vector<double> get_lidar(
        std::string_view robot_id = "robot", int count = 360,
        double max_range = 3.0, double fov = 2.0 * 3.14159265358979323846
    ) const = 0;

    virtual void set_display_path(const Path& path) = 0;
    virtual void set_planning_map(const Map& map) = 0;
};

class NavigationContext {
   public:
    virtual ~NavigationContext() = default;
};

std::unique_ptr<NavigationContext> nav_init(Simulator& sim);
void nav_run(Simulator& sim, NavigationContext& context);

}  // namespace pyrobo
