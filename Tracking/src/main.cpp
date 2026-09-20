#include <camera.hpp>
#include <gimbal.hpp>
#include <target.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace {

class PositionVelocityKalmanFilter {
public:
    using State = Eigen::Matrix<double, 6, 1>;
    using Covariance = Eigen::Matrix<double, 6, 6>;

    PositionVelocityKalmanFilter(const Eigen::Vector3d& initial_position,
                                 double measurement_stddev)
        : state_(State::Zero()),
          covariance_(Covariance::Identity()),
          measurement_noise_(Eigen::Matrix3d::Identity() *
                             measurement_stddev * measurement_stddev) {
        state_.head<3>() = initial_position;
        covariance_.topLeftCorner<3, 3>() =
            Eigen::Matrix3d::Identity() * 0.25;
        covariance_.bottomRightCorner<3, 3>() =
            Eigen::Matrix3d::Identity() * 25.0;
    }

    void predict(double dt) {
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        Eigen::Matrix<double, 6, 6> transition =
            Eigen::Matrix<double, 6, 6>::Identity();
        transition.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

        constexpr double acceleration_stddev = 8.0;
        const double acceleration_variance =
            acceleration_stddev * acceleration_stddev;
        Eigen::Matrix<double, 6, 6> process_noise =
            Eigen::Matrix<double, 6, 6>::Zero();
        process_noise.topLeftCorner<3, 3>() =
            Eigen::Matrix3d::Identity() * dt4 * acceleration_variance / 4.0;
        process_noise.topRightCorner<3, 3>() =
            Eigen::Matrix3d::Identity() * dt3 * acceleration_variance / 2.0;
        process_noise.bottomLeftCorner<3, 3>() =
            Eigen::Matrix3d::Identity() * dt3 * acceleration_variance / 2.0;
        process_noise.bottomRightCorner<3, 3>() =
            Eigen::Matrix3d::Identity() * dt2 * acceleration_variance;

        state_ = transition * state_;
        covariance_ = transition * covariance_ * transition.transpose() +
                      process_noise;
    }

    void update(const Eigen::Vector3d& measurement) {
        Eigen::Matrix<double, 3, 6> observation =
            Eigen::Matrix<double, 3, 6>::Zero();
        observation.leftCols<3>() = Eigen::Matrix3d::Identity();

        const Eigen::Vector3d innovation =
            measurement - observation * state_;
        const Eigen::Matrix3d innovation_covariance =
            observation * covariance_ * observation.transpose() +
            measurement_noise_;
        const Eigen::Matrix<double, 6, 3> kalman_gain =
            covariance_ * observation.transpose() *
            innovation_covariance.ldlt().solve(Eigen::Matrix3d::Identity());

        state_ += kalman_gain * innovation;

        const Covariance identity = Covariance::Identity();
        const Covariance correction = identity - kalman_gain * observation;
        covariance_ = correction * covariance_ * correction.transpose() +
                      kalman_gain * measurement_noise_ *
                          kalman_gain.transpose();
    }

    Eigen::Vector3d predicted_position(double dt) const {
        return state_.head<3>() + state_.tail<3>() * dt;
    }

private:
    State state_;
    Covariance covariance_;
    Eigen::Matrix3d measurement_noise_;
};

Eigen::Vector3d to_gimbal_position(const Eigen::Vector3d& camera_position,
                                   const Eigen::Matrix4d& transform) {
    Eigen::Vector4d homogeneous_position;
    homogeneous_position << camera_position, 1.0;
    return (transform * homogeneous_position).head<3>();
}

double normalize_angle(double angle) {
    constexpr double pi = 3.14159265358979323846;
    while (angle > pi) {
        angle -= 2.0 * pi;
    }
    while (angle < -pi) {
        angle += 2.0 * pi;
    }
    return angle;
}

}  // namespace

int main() {
    // 这里的初始化是必要的，请不要做更改参数以外的修改
    /*---------------------------------------------*/
    Gimbal gimbal;
    Camera camera;
    Target::ENABLE_DEBUG(Defs::DebugMode::GRAPHICAL);
    /*---------------------------------------------*/

    const Eigen::Matrix4d camera_to_gimbal = camera.get_transform();
    const Eigen::Vector3d initial_measurement =
        to_gimbal_position(camera.get_target_pos(), camera_to_gimbal);
    PositionVelocityKalmanFilter filter(initial_measurement, 0.05);

    constexpr double control_period = 0.001;
    constexpr double measurement_delay = 0.010;
    constexpr double gimbal_response = 0.8;
    constexpr double control_gain = 0.75;

    while (true) {
        const Eigen::Vector3d measurement =
            to_gimbal_position(camera.get_target_pos(), camera_to_gimbal);

        filter.predict(control_period);
        filter.update(measurement);

        // 相机观测约滞后 10 ms，用速度状态将估计外推到当前时刻。
        const Eigen::Vector3d estimated_position =
            filter.predicted_position(measurement_delay);
        const double horizontal_distance =
            std::hypot(estimated_position.x(), estimated_position.y());

        if (estimated_position.squaredNorm() > 1e-12) {
            const double target_yaw =
                std::atan2(estimated_position.y(), estimated_position.x());
            const double target_pitch =
                std::atan2(estimated_position.z(), horizontal_distance);

            const auto [current_yaw, current_pitch] =
                gimbal.get_curr_angle();
            const double yaw_error =
                normalize_angle(target_yaw - current_yaw);
            const double pitch_error = target_pitch - current_pitch;

            gimbal.set_target({
                control_gain * yaw_error / gimbal_response,
                control_gain * pitch_error / gimbal_response
            });
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }

    return 0;
}
