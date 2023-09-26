#include "huron/odrive/odrive_rotary_encoder.h"
namespace huron {
namespace odrive {

ODriveEncoder::ODriveEncoder(float cpr,
                 std::shared_ptr<can::ODrive> odrive)
  : huron::RotaryEncoder(cpr), odrive_(std::move(odrive)) {}

void ODriveEncoder::Configure() {
}

void ODriveEncoder::Initialize() {
}

void ODriveEncoder::SetUp() {
}

void ODriveEncoder::Terminate() {
}


float ODriveEncoder::GetPosition() {
  prev_count_ = count_;
  float pos, vel;
  odrive_->GetEncoderEstimates(pos, vel);
  count_ = pos * cpr_;
  velocity_ = vel * cpr_;
  return count_;
}

float ODriveEncoder::GetVelocity() {
  GetPosition();
  return velocity_;
}

}  // namespace odrive
}  // namespace huron
