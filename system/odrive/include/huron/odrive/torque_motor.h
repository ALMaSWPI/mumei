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

  void Configure() override;
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  bool Move(float value) override;
  bool Move(const std::vector<float>& values) override;
  bool Stop() override;

 private:
  std::shared_ptr<HuronODriveCAN> odrive_;
};

}// namespace odrive
}// namespace huron
