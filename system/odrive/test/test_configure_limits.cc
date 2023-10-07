#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "huron/control_interfaces/revolute_joint.h"
#include "huron/driver/can/socket_can_bus.h"

#include "huron/odrive/odrive_rotary_encoder.h"
#include "huron/odrive/odrive_torque_motor.h"
#include "huron/utils/time.h"

const float kGearRatio1 = 1.0;
const float kGearRatio2 = 80.0;
const float kCPR = 4096.0;

int main(int argc, char* argv[]) {
  // TODO(dtbpkmte): make pointer to hcb unique_ptr
  huron::driver::can::SocketCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb, 0, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());
  huron::RevoluteJoint left_knee_joint{
    std::make_unique<huron::odrive::TorqueMotor>(left_knee_odrive),
    std::make_unique<huron::odrive::ODriveEncoder>(kCPR, left_knee_odrive),
    kGearRatio1, kGearRatio2};

  left_knee_joint.GetMotor().GetDriver().Configure("velocity_limit", 15.0);
  left_knee_joint.GetMotor().GetDriver().Configure("current_limit", 70.0);
  left_knee_joint.GetEncoder().Configure("cpr", 8192);

  // Manually check "velocity_limit" in ODrive
}
