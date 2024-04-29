#include <chrono>
#include <thread>

#include "mumei/odrive/odrive_enums.h"
#include "mumei/exceptions/not_implemented_exception.h"
#include "mumei/odrive/odrive_torque_motor.h"

namespace mumei {
namespace odrive {

TorqueMotor::TorqueMotor(
  const std::string& name,
  std::unique_ptr<TorqueMotorConfiguration> config,
  std::shared_ptr<ODrive> odrive,
  double gear_ratio)
  : mumei::TorqueMotor(name, std::move(config), gear_ratio),
    odrive_(std::move(odrive)) {}

TorqueMotor::TorqueMotor(
  const std::string& name,
  std::shared_ptr<ODrive> odrive,
  double gear_ratio)
  : mumei::TorqueMotor(name, gear_ratio),
    odrive_(std::move(odrive)) {}

TorqueMotor::TorqueMotor(
  const std::string& name,
  std::shared_ptr<ODrive> odrive)
  : mumei::TorqueMotor(name),
    odrive_(std::move(odrive)) {}

void TorqueMotor::Initialize() {
  odrive_->Initialize();
  // Set input & control modes
  odrive_->SetControllerModes(CONTROL_MODE_TORQUE_CONTROL,
                              INPUT_MODE_PASSTHROUGH);
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void TorqueMotor::SetUp() {
  // Set axis state CLOSED_LOOP
  odrive_->SetAxisRequestedState(
      AXIS_STATE_CLOSED_LOOP_CONTROL);
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void TorqueMotor::Terminate() {
  odrive_->SetAxisRequestedState(AXIS_STATE_IDLE);
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

bool TorqueMotor::Move(double value) {
  return odrive_->SetInputTorque(value);
}

bool TorqueMotor::Move(const std::vector<double>& values) {
  assert(values.size() == 1);
  return odrive_->SetInputTorque(values[0]);
}

bool TorqueMotor::Move(const Eigen::VectorXd& values) {
  assert(values.size() == 1);
  return odrive_->SetInputTorque(values[0]);
}

bool TorqueMotor::Stop() {
  return odrive_->SetInputTorque(0);
}

}  // namespace odrive
}  // namespace mumei
