#include "huron/odrive/encoder.h"

namespace huron {
namespace odrive {

Encoder::Encoder(float cpr,
                 std::shared_ptr<can::ODrive> odrive)
  : huron::Encoder(cpr), odrive_(std::move(odrive)) {}

void Encoder::Configure() {
}

void Encoder::Initialize() {
}

void Encoder::SetUp() {
}

void Encoder::Terminate() {
}


float Encoder::GetCount() {
  prev_count_ = count_;
  float pos, vel;
  odrive_->GetEncoderEstimates(pos, vel);
  count_ = pos * cpr_;
  velocity_ = vel * cpr_;
  return count_;
}

float Encoder::GetVelocity() {
  GetCount();
  return velocity_;
}

}  // namespace odrive
}  // namespace huron
