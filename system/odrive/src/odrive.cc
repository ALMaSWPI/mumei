#include <chrono>
#include <thread>

#include "huron/odrive/odrive.h"

namespace huron {
namespace odrive {

bool ODrive::Initialize() {
  // Set axis state IDLE
  SetAxisRequestedState(AXIS_STATE_IDLE);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Calibrate
  if (!is_calibrated_) {
    return Calibrate();
  }
  return true;
}

bool ODrive::Calibrate() {
  SetAxisRequestedState(
      AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
  std::this_thread::sleep_for(std::chrono::seconds(25));
  is_calibrated_ = true;
  return true;
}

}  // namespace odrive
}  // namespace huron
