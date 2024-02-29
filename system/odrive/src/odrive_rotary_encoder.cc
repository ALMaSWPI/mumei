#include "huron/odrive/odrive_rotary_encoder.h"

namespace huron {
namespace odrive {

ODriveEncoder::ODriveEncoder(double gear_ratio,
                             std::unique_ptr<RotaryEncoderConfiguration<double>> config,
                             std::shared_ptr<ODrive> odrive)
    : huron::RotaryEncoder<double>(gear_ratio,
                           std::move(config)),
      odrive_(std::move(odrive)) {}

ODriveEncoder::ODriveEncoder(double gear_ratio,
                             double cpr,
                             std::shared_ptr<ODrive> odrive)
    : huron::RotaryEncoder<double>(gear_ratio, cpr),
      odrive_(std::move(odrive)) {}

ODriveEncoder::ODriveEncoder(double cpr, std::shared_ptr<ODrive> odrive)
  : ODriveEncoder(1.0, cpr, std::move(odrive)) {}


void ODriveEncoder::Initialize() {
}

void ODriveEncoder::SetUp() {
}

void ODriveEncoder::Terminate() {
}


void ODriveEncoder::DoUpdateState() {
  float pos, vel;
  odrive_->GetEncoderEstimates(pos, vel);
  count_ = pos * cpr_;
  velocity_ = vel * cpr_;
}

}  // namespace odrive
}  // namespace huron
