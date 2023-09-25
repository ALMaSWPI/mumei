#pragma once

#include <memory>

#include "huron/control_interfaces/encoder.h"
#include "huron/driver/can/huron_odrive_can.h"

namespace huron {
namespace odrive {

class Encoder : public huron::Encoder {
 public:
  explicit Encoder(float cpr, std::shared_ptr<can::ODrive> odrive);
  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  ~Encoder() = default;

  void Configure() override;
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  float GetCount() override;
  float GetVelocity() override;

 private:
  std::shared_ptr<can::ODrive> odrive_;
};

}  // namespace odrive
}  // namespace huron
