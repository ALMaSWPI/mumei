#include <chrono>
#include <thread>

#include "huron/odrive/odrive.h"

namespace huron {
namespace odrive {

void ODrive::Initialize() {
  // Set axis state IDLE
  SetAxisRequestedState(AXIS_STATE_IDLE);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Calibrate
  if (!is_calibrated_) {
    Calibrate();
  }
}

bool ODrive::Calibrate() {
  SetAxisRequestedState(
      AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
  std::this_thread::sleep_for(std::chrono::seconds(25));
  is_calibrated_ = true;
  return true;
}

void ODrive::ConfigureKey(std::string config_key, std::any config_value) {
  float value = std::any_cast<float>(config_value);
  if (config_key == "velocity_limit") {
    SetLimits(value, std::any_cast<float>(config_->Get("current_limit")));
  } else if (config_key == "current_limit") {
    SetLimits(std::any_cast<float>(config_->Get("velocity_limit")), value);
  } else if (config_key == "traj_vel_limit") {
    SetTrajVelLimit(value);
  } else if (config_key == "traj_accel_limit") {
    SetTrajAccelLimits(value,
                       std::any_cast<float>(config_->Get("traj_decel_limit")));
  } else if (config_key == "traj_decel_limit") {
    SetTrajAccelLimits(std::any_cast<float>(config_->Get("traj_accel_limit")),
                       value);
  } else if (config_key == "traj_inertia") {
    SetTrajInertia(value);
  }
}

}  // namespace odrive
}  // namespace huron
