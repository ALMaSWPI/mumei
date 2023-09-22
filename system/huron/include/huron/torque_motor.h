#pragma once

#include "huron/control_interfaces/motor.h"

namespace huron {

  class TorqueMotor : public Motor {

   public:
    explicit TorqueMotor();

   private:
    float current_limit_ = 0;
    float velocity_limit_ = 0;
    float desired_value_ = 0;
  };

}// namespace huron
