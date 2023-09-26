#include "huron/control_interfaces/joint.h"

namespace huron {

Joint::Joint(std::unique_ptr<Motor> motor,
             std::unique_ptr<RotaryEncoder> encoder)
    : motor_(std::move(motor)),
      encoder_(std::move(encoder)) {}

void Joint::Configure() {
  encoder_->Configure();
  motor_->Configure();
}

void Joint::Initialize() {
  encoder_->Initialize();
  motor_->Initialize();
}

void Joint::SetUp() {
  encoder_->SetUp();
  motor_->SetUp();
}

void Joint::Terminate() {
  encoder_->Terminate();
  motor_->Terminate();
}

bool Joint::Move(float value) {
  return motor_->Move(value);
}

bool Joint::Move(const std::vector<float>& values) {
  return motor_->Move(values);
}

bool Joint::Stop() {
  return motor_->Stop();
}

}  // namespace huron
