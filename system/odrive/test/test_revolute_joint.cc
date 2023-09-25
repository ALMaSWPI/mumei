#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "huron/control_interfaces/revolute_joint.h"
#include "huron/driver/can/huron_canbus.h"
#include "huron/odrive/torque_motor.h"
#include "huron/odrive/encoder.h"
#include "huron/utils/time.h"

const float kGearRatio1 = 2.0;
const float kGearRatio2 = 40.0;
const float kCPR = 4096.0;

int main(int argc, char* argv[]) {
  // TODO(dtbpkmte): make pointer to hcb unique_ptr
  HURONCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<HuronODriveCAN>(
    &hcb, 0);
  huron::RevoluteJoint left_knee_joint{
    std::make_unique<huron::odrive::TorqueMotor>(left_knee_odrive),
    std::make_unique<huron::odrive::Encoder>(kCPR, left_knee_odrive),
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
  left_knee_joint.Move(0.3);

  while (since(start_time).count() < 3 /* seconds */) {
    std::cout << "Current position: "
              << left_knee_joint.GetPosition()
              << std::endl;
    std::cout << "Current velocity: "
              << left_knee_joint.GetVelocity()
              << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::cout << "Stopping..." << std::endl;
  left_knee_joint.Stop();

  std::cout << "Final position: "
            << left_knee_joint.GetPosition()
            << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(5));

  left_knee_joint.Terminate();
  std::cout << "Terminated." << std::endl;
}
