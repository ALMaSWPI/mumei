#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "mumei/control_interfaces/revolute_joint.h"
#include "mumei/driver/can/socket_can_bus.h"

#include "mumei/odrive/odrive_rotary_encoder.h"
#include "mumei/odrive/odrive_torque_motor.h"
#include "mumei/utils/time.h"

const float kGearRatio1 = 1.0;
const float kGearRatio2 = 80.0;
const float kCPR = 4096.0;

int main(int argc, char* argv[]) {
  // TODO(dtbpkmte): make pointer to hcb unique_ptr
  mumei::driver::can::SocketCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<mumei::odrive::ODriveCAN>(
    &hcb, 0, std::make_unique<mumei::odrive::ODrive::ODriveConfiguration>());
  mumei::RevoluteJoint left_knee_joint{
    std::make_unique<mumei::odrive::TorqueMotor>(left_knee_odrive),
    std::make_unique<mumei::odrive::ODriveEncoder>(kCPR, left_knee_odrive),
    kGearRatio1, kGearRatio2};

  auto start_time = std::chrono::steady_clock::now();
  while (since(start_time).count() < 3000 /* ms */) {
    std::cout << "Current position: "
              << left_knee_joint.GetPosition()
              << std::endl;
    std::cout << "Current velocity: "
              << left_knee_joint.GetVelocity()
              << std::endl;
  }
}
