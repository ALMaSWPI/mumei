#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "mumei/control_interfaces/revolute_joint.h"
#include "mumei/driver/can/socket_can_bus.h"

#include "mumei/odrive/odrive_rotary_encoder.h"
#include "mumei/odrive/odrive_torque_motor.h"
#include "mumei/utils/time.h"

using namespace std::chrono_literals;  //NOLINT

const float kGearRatio1 = 1.0;
const float kGearRatio2 = 80.0;
const float kCPR = 4096.0;

int main(int argc, char* argv[]) {
  // First, manually set these values to ODrive
  mumei::ConfigMap odrive_config{
    {"velocity_limit", 20.0f},
    {"current_limit", 50.0f},
  };
  // TODO(dtbpkmte): make pointer to hcb unique_ptr
  mumei::driver::can::SocketCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<mumei::odrive::ODriveCAN>(
      &hcb,
      0,
      std::make_unique<mumei::odrive::ODrive::ODriveConfiguration>(
        odrive_config));
  mumei::RevoluteJoint left_knee_joint{
    std::make_unique<mumei::odrive::TorqueMotor>(left_knee_odrive),
    std::make_unique<mumei::odrive::ODriveEncoder>(kCPR, left_knee_odrive),
    kGearRatio1, kGearRatio2};

  std::cout << "Configuration starting...\n";
  left_knee_joint.GetMotor().GetDriver().Configure("velocity_limit", 15.0f);
  std::this_thread::sleep_for(1s);
  std::cout << "Configured velocity_limit\n";
  left_knee_joint.GetMotor().GetDriver().Configure("current_limit", 70.0f);
  std::this_thread::sleep_for(1s);
  std::cout << "Configured curent_limit\n";

  // Manually check "velocity_limit" and "current_limit" in ODrive
}
