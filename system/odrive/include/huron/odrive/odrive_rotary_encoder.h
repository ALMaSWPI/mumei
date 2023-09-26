#pragma once

#include <memory>
#include "huron/control_interfaces/rotary_encoder.h"
#include "huron/driver/can/huron_odrive_can.h"

namespace huron {
namespace odrive {

class ODriveEncoder : public huron::RotaryEncoder {
 public:
  explicit ODriveEncoder(float cpr, std::shared_ptr<can::ODrive> odrive);
  ODriveEncoder(const ODriveEncoder&) = delete;
  ODriveEncoder& operator=(const ODriveEncoder&) = delete;
  ~ODriveEncoder() = default;

  void Configure() override;
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  float GetPosition() override;
  float GetVelocity() override;
 private:
  std::shared_ptr<ODrive> odrive_;
};

}  // namespace odrive
}  // namespace huron
