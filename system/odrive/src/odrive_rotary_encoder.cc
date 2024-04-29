#include "huron/odrive/odrive_rotary_encoder.h"

namespace huron {
namespace odrive {

ODriveEncoder::ODriveEncoder(const std::string& name,
                             double gear_ratio,
                             std::unique_ptr<RotaryEncoderConfiguration> config,
                             std::shared_ptr<ODrive> odrive)
    : huron::RotaryEncoder(name, gear_ratio,
                           std::move(config)),
      odrive_(std::move(odrive)) {}

ODriveEncoder::ODriveEncoder(const std::string& name,
                             double gear_ratio,
                             double cpr,
                             std::shared_ptr<ODrive> odrive)
    : huron::RotaryEncoder(name, gear_ratio, cpr),
      odrive_(std::move(odrive)) {}

ODriveEncoder::ODriveEncoder(const std::string& name,
                             double cpr, std::shared_ptr<ODrive> odrive)
  : ODriveEncoder(name, 1.0, cpr, std::move(odrive)) {}


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
