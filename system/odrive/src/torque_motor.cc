#include "huron/exceptions/NotImplementedException.h"
#include "huron/odrive/torque_motor.h"

namespace huron {
namespace odrive {

TorqueMotor::TorqueMotor(std::shared_ptr<HuronODriveCAN> odrive)
  : odrive_(std::move(odrive)) {}

bool TorqueMotor::Move(float value) {
  return odrive_->SetInputTorque(value);
}

bool TorqueMotor::Move(std::vector<float> values) {
  throw NotImplementedException();
}


}// namespace odrive
}// namespace huron
