#pragma once

#include "huron/control_interfaces/motor.h"

namespace huron {

class PositionMotor : public Motor {

 public:
  PositionMotor() = default;
  PositionMotor(const PositionMotor&) = delete;
  PositionMotor& operator=(const PositionMotor&) = delete;
  ~PositionMotor() = default;

};

}//namespace huron
