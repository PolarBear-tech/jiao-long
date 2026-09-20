#pragma once
#include "defs.hpp"

class Gimbal {
 public:
  Gimbal();
  ~Gimbal();
  Defs::DirAngle get_curr_angle();
  void set_target(Defs::DirAngle delta_angle);

 private:
  class Impl;
  Impl* impl_;
};