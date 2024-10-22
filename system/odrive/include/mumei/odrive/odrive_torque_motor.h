#pragma once

#include <memory>
#include <vector>
#include <string>

#include "mumei/control_interfaces/torque_motor.h"
#include "mumei/odrive/odrive_can.h"

namespace mumei {
namespace odrive {

class TorqueMotor : public mumei::TorqueMotor {
 public:
  TorqueMotor(
    const std::string& name,
    std::unique_ptr<TorqueMotorConfiguration> config,
    std::shared_ptr<ODrive> odrive,
    double gear_ratio);
  TorqueMotor(
    const std::string& name,
    std::shared_ptr<ODrive> odrive,
    double gear_ratio);
  TorqueMotor(const std::string& name, std::shared_ptr<ODrive> odrive);
  TorqueMotor(const TorqueMotor&) = delete;
  TorqueMotor& operator=(const TorqueMotor&) = delete;
  ~TorqueMotor() = default;

  // GenericComponent methods
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  bool Move(double value) override;
  bool Move(const std::vector<double>& values) override;
  bool Move(const Eigen::VectorXd& values) override;
  bool Stop() override;

 private:
  std::shared_ptr<ODrive> odrive_;
};

}  // namespace odrive
}  // namespace mumei
