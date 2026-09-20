#ifndef DEFS_HPP
#define DEFS_HPP

#include <Eigen/Dense>

namespace Defs {
  struct DirAngle {
    double yaw;
    double pitch;
    DirAngle operator+(const DirAngle& other) const {
      return {yaw + other.yaw, pitch + other.pitch};
    }
    DirAngle operator-(const DirAngle& other) const {
      return {yaw - other.yaw, pitch - other.pitch};
    }
    DirAngle operator*(double scalar) const {
      return {yaw * scalar, pitch * scalar};
    }
  };
  using PositionVec = Eigen::Vector3d;
  using RotationMat = Eigen::Matrix3d;
  using TransformMat = Eigen::Matrix4d;
  enum class DroneTarget { PATROL, OUTPOST, BASE };
  enum class DebugMode { ESSENTIAL = 0, DETAIL = 1, GRAPHICAL = 2, ALL = 3 };
}
#endif  // DEFS_HPP
