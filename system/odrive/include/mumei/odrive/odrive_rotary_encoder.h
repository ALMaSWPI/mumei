#pragma once

#include <memory>
#include <string>
#include "mumei/control_interfaces/rotary_encoder.h"
#include "mumei/odrive/odrive.h"

namespace mumei {
namespace odrive {

class ODriveEncoder final : public mumei::RotaryEncoder {
 public:
  ODriveEncoder(const std::string& name,
                double gear_ratio,
                std::unique_ptr<RotaryEncoderConfiguration> config,
                std::shared_ptr<ODrive> odrive);
  ODriveEncoder(const std::string& name,
                double gear_ratio, double cpr, std::shared_ptr<ODrive> odrive);
  ODriveEncoder(const std::string& name,
                double cpr, std::shared_ptr<ODrive> odrive);
  ODriveEncoder(const ODriveEncoder&) = delete;
  ODriveEncoder& operator=(const ODriveEncoder&) = delete;
  ~ODriveEncoder() override = default;

  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

 protected:
  void DoUpdateState() override;

 private:
  std::shared_ptr<ODrive> odrive_;
};

}  // namespace odrive
}  // namespace mumei
