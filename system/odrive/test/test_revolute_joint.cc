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

  std::cout << "Initializing..." << std::endl;
  left_knee_joint.Initialize();
  std::cout << "Initialization completed." << std::endl;

  std::cout << "Enabling motor..." << std::endl;
  left_knee_joint.SetUp();
  std::cout << "Motor enabled." << std::endl;

  std::cout << "Initial position: "
            << left_knee_joint.GetPosition()
            << std::endl;

  std::cout << "Moving..." << std::endl;
  auto start_time = std::chrono::steady_clock::now();
  left_knee_joint.Move(0.2);
  while (since(start_time).count() < 3000 /* ms */) {
    std::cout << "Current position: "
              << left_knee_joint.GetPosition()
              << std::endl;
    std::cout << "Current velocity: "
              << left_knee_joint.GetVelocity()
              << std::endl;
  }

  std::cout << "Stopping..." << std::endl;
  left_knee_joint.Stop();

  std::cout << "Final position: "
            << left_knee_joint.GetPosition()
            << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(5));

  left_knee_joint.Terminate();
  std::cout << "Terminated." << std::endl;
}
