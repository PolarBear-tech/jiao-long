#pragma once
#include <memory>

#include "defs.hpp"

class Camera {
 public:
  using Position = Defs::PositionVec;
  using Transform = Defs::TransformMat;

  Camera();
  ~Camera();
  Position get_target_pos() const;
  Transform get_transform() const;  // 返回 4x4 齐次变换矩阵

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
