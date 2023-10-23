#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "huron/control_interfaces/revolute_joint.h"
#include "huron/driver/can/socket_can_bus.h"

#include "huron/odrive/odrive_rotary_encoder.h"
#include "huron/odrive/odrive_torque_motor.h"
#include "huron/utils/time.h"

using namespace std::chrono_literals;  //NOLINT

const float kGearRatio1 = 1.0;
const float kGearRatio2 = 80.0;
const float kCPR = 4096.0;

int main(int argc, char* argv[]) {
  // First, manually set these values to ODrive
  huron::ConfigMap odrive_config{
    {"velocity_limit", 20.0f},
    {"current_limit", 50.0f},
  };
  // TODO(dtbpkmte): make pointer to hcb unique_ptr
  huron::driver::can::SocketCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb, 0, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>(odrive_config));
  huron::RevoluteJoint left_knee_joint{
    std::make_unique<huron::odrive::TorqueMotor>(left_knee_odrive),
    std::make_unique<huron::odrive::ODriveEncoder>(kCPR, left_knee_odrive),
    kGearRatio1, kGearRatio2};

  std::cout << "Configuration starting...\n";
  left_knee_joint.GetMotor().GetDriver().Configure("velocity_limit", 15.0f);
  std::this_thread::sleep_for(0.5s);
  std::cout << "Configured velocity_limit\n";
  left_knee_joint.GetMotor().GetDriver().Configure("current_limit", 70.0f);
  std::this_thread::sleep_for(0.5s);
  std::cout << "Configured curent_limit\n";

  // Manually check "velocity_limit" and "current_limit" in ODrive
}
