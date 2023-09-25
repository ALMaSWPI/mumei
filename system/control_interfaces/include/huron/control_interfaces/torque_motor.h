#pragma once

#include "huron/control_interfaces/motor.h"

namespace huron {

class TorqueMotor : public Motor {
 public:
  TorqueMotor() = default;
  TorqueMotor(const TorqueMotor&) = delete;
  TorqueMotor& operator=(const TorqueMotor&) = delete;
  ~TorqueMotor() = default;

};

}  //namespace huron
