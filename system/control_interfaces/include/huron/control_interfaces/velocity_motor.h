#pragma once

#include "huron/control_interfaces/motor.h"

namespace huron {

class VelocityMotor : public Motor {

 public:
  VelocityMotor() = default;
  VelocityMotor(const VelocityMotor&) = delete;
  VelocityMotor& operator=(const VelocityMotor&) = delete;
  ~VelocityMotor() = default;

};

}//namespace huron
