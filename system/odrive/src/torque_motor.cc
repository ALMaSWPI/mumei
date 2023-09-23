#include <chrono>
#include <thread>

#include "huron/exceptions/NotImplementedException.h"
#include "huron/odrive/torque_motor.h"
#include "huron/driver/can/ODriveEnums.h"

using namespace std::chrono_literals;

namespace huron {
namespace odrive {

TorqueMotor::TorqueMotor(std::shared_ptr<HuronODriveCAN> odrive)
  : odrive_(std::move(odrive)) {}

void TorqueMotor::Configure() {

}

void TorqueMotor::Initialize() {
  // Set axis state IDLE
  odrive_->SetAxisRequestedState(AXIS_STATE_IDLE);
  std::this_thread::sleep_for(1s);

  // Set input & control modes
  odrive_->SetControllerModes(CONTROL_MODE_TORQUE_CONTROL,
                              INPUT_MODE_PASSTHROUGH);
  std::this_thread::sleep_for(1s);

  // Calibrate
  odrive_->SetAxisRequestedState(
      AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
  std::this_thread::sleep_for(25s);
}

void TorqueMotor::SetUp() {
  // Set axis state CLOSED_LOOP
  odrive_->SetAxisRequestedState(
      AXIS_STATE_CLOSED_LOOP_CONTROL);
  std::this_thread::sleep_for(1s);
}

void TorqueMotor::Terminate() {
  odrive_->SetAxisRequestedState(AXIS_STATE_IDLE);
  std::this_thread::sleep_for(1s);
}

bool TorqueMotor::Move(float value) {
  return odrive_->SetInputTorque(value);
}

bool TorqueMotor::Move(const std::vector<float>& values) {
  throw NotImplementedException();
}

bool TorqueMotor::Stop() {
  return odrive_->SetInputTorque(0);
}


}// namespace odrive
}// namespace huron
