#pragma once

#include <memory>

#include "huron/control_interfaces/torque_motor.h"
#include "huron/driver/can/huron_odrive_can.h"

namespace huron {
namespace odrive {

class TorqueMotor : public huron::TorqueMotor {

 public:
  explicit TorqueMotor(std::shared_ptr<HuronODriveCAN> odrive);
  TorqueMotor(const TorqueMotor&) = delete;
  TorqueMotor& operator=(const TorqueMotor&) = delete;
  ~TorqueMotor() = default;

  bool Move(float value) override;
  bool Move(std::vector<float> values) override;

 private:
  std::shared_ptr<HuronODriveCAN> odrive_;
};

}// namespace odrive
}// namespace huron
