#include "pyrobo/c_api.h"

#include "pyrobo/interface.hpp"

#include <algorithm>
#include <cstring>
#include <exception>
#include <memory>
#include <string>

namespace {

void write_error(char* buffer, std::size_t capacity, const char* message) noexcept {
    if (buffer == nullptr || capacity == 0) {
        return;
    }
    const char* text = message == nullptr ? "unknown error" : message;
    const std::size_t length = std::min(std::strlen(text), capacity - 1);
    std::memcpy(buffer, text, length);
    buffer[length] = '\0';
}

class CallbackSimulator final : public pyrobo::Simulator {
public:
    explicit CallbackSimulator(const pyrobo_callbacks& callbacks) : callbacks_(callbacks) {
        if (callbacks_.get_map == nullptr) {
            throw std::invalid_argument("get_map callback is required");
        }

        int height = 0;
        int width = 0;
        double resolution = 0.0;
        double origin_x = 0.0;
        double origin_y = 0.0;
        const std::uint8_t* data = nullptr;
        std::size_t data_size = 0;
        check(
            callbacks_.get_map(
                callbacks_.user_data,
                &height,
                &width,
                &resolution,
                &origin_x,
                &origin_y,
                &data,
                &data_size
            ),
            "get_map"
        );
        if (data == nullptr) {
            throw std::runtime_error("get_map returned null data");
        }
        map_ = pyrobo::Map(
            height,
            width,
            resolution,
            {origin_x, origin_y},
            std::vector<std::uint8_t>(data, data + data_size)
        );
    }

    const pyrobo::Map& map() const override { return map_; }

    pyrobo::Pose get_pose(std::string_view robot_id) const override {
        require(callbacks_.get_pose, "get_pose");
        pyrobo::Pose pose;
        const std::string id(robot_id);
        check(callbacks_.get_pose(callbacks_.user_data, id.c_str(), &pose.x, &pose.y, &pose.yaw), "get_pose");
        return pose;
    }

    pyrobo::RobotInfo get_robot(std::string_view robot_id) const override {
        require(callbacks_.get_robot, "get_robot");
        pyrobo::RobotInfo robot;
        const std::string id(robot_id);
        check(callbacks_.get_robot(callbacks_.user_data, id.c_str(), &robot.radius), "get_robot");
        return robot;
    }

    std::optional<pyrobo::Pose> get_goal() const override {
        require(callbacks_.get_goal, "get_goal");
        pyrobo::Pose goal;
        const int present = callbacks_.get_goal(
            callbacks_.user_data, &goal.x, &goal.y, &goal.yaw
        );
        if (present < 0) {
            throw std::runtime_error("get_goal callback failed");
        }
        return present == 0 ? std::nullopt : std::optional<pyrobo::Pose>(goal);
    }

    pyrobo::Feedback get_feedback(std::string_view robot_id) const override {
        require(callbacks_.get_feedback, "get_feedback");
        const std::string id(robot_id);
        double first = 0.0;
        double second = 0.0;
        check(
            callbacks_.get_feedback(callbacks_.user_data, id.c_str(), &first, &second),
            "get_feedback"
        );
        return {first, second};
    }

    void set_control(double first, double second, std::string_view robot_id) override {
        require(callbacks_.set_control, "set_control");
        const std::string id(robot_id);
        check(
            callbacks_.set_control(callbacks_.user_data, id.c_str(), first, second),
            "set_control"
        );
    }

    pyrobo::Control get_control(std::string_view robot_id) const override {
        require(callbacks_.get_control, "get_control");
        const std::string id(robot_id);
        double first = 0.0;
        double second = 0.0;
        check(
            callbacks_.get_control(callbacks_.user_data, id.c_str(), &first, &second),
            "get_control"
        );
        return {first, second};
    }

    std::vector<double> get_lidar(
        std::string_view robot_id,
        int count,
        double max_range,
        double fov
    ) const override {
        require(callbacks_.get_lidar, "get_lidar");
        if (count <= 0) {
            throw std::invalid_argument("lidar count must be positive");
        }
        const std::string id(robot_id);
        std::vector<double> output(static_cast<std::size_t>(count));
        std::size_t output_size = 0;
        check(
            callbacks_.get_lidar(
                callbacks_.user_data,
                id.c_str(),
                count,
                max_range,
                fov,
                output.data(),
                output.size(),
                &output_size
            ),
            "get_lidar"
        );
        if (output_size > output.size()) {
            throw std::runtime_error("get_lidar returned too many values");
        }
        output.resize(output_size);
        return output;
    }

    void set_display_path(const pyrobo::Path& path) override {
        require(callbacks_.set_display_path, "set_display_path");
        std::vector<pyrobo_point> points;
        points.reserve(path.size());
        for (const auto& point : path) {
            points.push_back({point.x, point.y});
        }
        check(
            callbacks_.set_display_path(callbacks_.user_data, points.data(), points.size()),
            "set_display_path"
        );
    }

    void set_planning_map(const pyrobo::Map& map) override {
        require(callbacks_.set_planning_map, "set_planning_map");
        const auto shape = map.shape();
        const auto origin = map.origin();
        const auto& data = map.data();
        check(
            callbacks_.set_planning_map(
                callbacks_.user_data,
                shape.height,
                shape.width,
                map.resolution(),
                origin.x,
                origin.y,
                data.data(),
                data.size()
            ),
            "set_planning_map"
        );
    }

private:
    template <typename Callback>
    static void require(Callback callback, const char* name) {
        if (callback == nullptr) {
            throw std::runtime_error(std::string(name) + " callback is not available");
        }
    }

    static void check(int result, const char* name) {
        if (result == 0) {
            throw std::runtime_error(std::string(name) + " callback failed");
        }
    }

    pyrobo_callbacks callbacks_{};
    pyrobo::Map map_;
};

struct NavigationHandle {
    explicit NavigationHandle(const pyrobo_callbacks& callbacks)
        : simulator(callbacks), context(pyrobo::nav_init(simulator)) {}

    CallbackSimulator simulator;
    std::unique_ptr<pyrobo::NavigationContext> context;
};

}  // namespace

extern "C" PYROBO_API void* pyrobo_create_navigation(
    const pyrobo_callbacks* callbacks,
    char* error_message,
    std::size_t error_capacity
) {
    try {
        if (callbacks == nullptr) {
            throw std::invalid_argument("callbacks must not be null");
        }
        return new NavigationHandle(*callbacks);
    } catch (const std::exception& error) {
        write_error(error_message, error_capacity, error.what());
        return nullptr;
    } catch (...) {
        write_error(error_message, error_capacity, "unknown C++ exception");
        return nullptr;
    }
}

extern "C" PYROBO_API int pyrobo_run_navigation(
    void* navigation,
    char* error_message,
    std::size_t error_capacity
) {
    try {
        if (navigation == nullptr) {
            throw std::invalid_argument("navigation handle is null");
        }
        auto* handle = static_cast<NavigationHandle*>(navigation);
        pyrobo::nav_run(handle->simulator, *handle->context);
        return 1;
    } catch (const std::exception& error) {
        write_error(error_message, error_capacity, error.what());
        return 0;
    } catch (...) {
        write_error(error_message, error_capacity, "unknown C++ exception");
        return 0;
    }
}

extern "C" PYROBO_API void pyrobo_destroy_navigation(void* navigation) {
    delete static_cast<NavigationHandle*>(navigation);
}
