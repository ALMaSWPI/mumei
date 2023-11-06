#include <iostream>

#include "huron/control_interfaces/robot.h"
#include "huron/control_interfaces/revolute_joint.h"
#include "huron/driver/can/socket_can_bus.h"
#include "huron/odrive/odrive_rotary_encoder.h"
#include "huron/odrive/odrive_torque_motor.h"
#include "huron/utils/time.h"

const float kGearRatio1 = 1.0;
const float kGearRatio2 = 80.0;
const float kCPR = 4096.0;

// Helper function to print vectors
template<typename T>
void PrintVector(const std::vector<T>& vec) {
  for (auto& e : vec) {
    std::cout << e << ' ';
  }
  std::cout << '\n';
}

int main(int argc, char* argv[]) {
  using namespace huron;

  Robot huron;

  huron::driver::can::SocketCanBus hcb0{"can0", 0};
  huron::driver::can::SocketCanBus hcb1{"can0", 1};
  huron::driver::can::SocketCanBus hcb6{"can1", 6};
  huron::driver::can::SocketCanBus hcb7{"can1", 7};
  auto left_knee_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb0, 0, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());
  auto left_hip_pitch_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb1, 1, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());
  auto right_knee_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb6, 6, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());
  auto right_hip_pitch_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb7, 7, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());

  huron.AddJoint(std::make_shared<huron::RevoluteJoint>(
                   std::make_unique<huron::odrive::TorqueMotor>(
                     left_knee_odrive),
                   std::make_unique<huron::odrive::ODriveEncoder>(
                     kCPR, left_knee_odrive),
                   kGearRatio1,
                   kGearRatio2
                 ));
  huron.AddJoint(std::make_shared<huron::RevoluteJoint>(
                   std::make_unique<huron::odrive::TorqueMotor>(
                     left_hip_pitch_odrive),
                   std::make_unique<huron::odrive::ODriveEncoder>(
                     kCPR, left_hip_pitch_odrive),
                   kGearRatio1,
                   kGearRatio2
                 ));
  huron.AddJoint(std::make_shared<huron::RevoluteJoint>(
                   std::make_unique<huron::odrive::TorqueMotor>(
                     right_knee_odrive),
                   std::make_unique<huron::odrive::ODriveEncoder>(
                     kCPR, right_knee_odrive),
                   kGearRatio1,
                   kGearRatio2
                 ));
  huron.AddJoint(std::make_shared<huron::RevoluteJoint>(
                   std::make_unique<huron::odrive::TorqueMotor>(
                     right_hip_pitch_odrive),
                   std::make_unique<huron::odrive::ODriveEncoder>(
                     kCPR, right_hip_pitch_odrive),
                   kGearRatio1,
                   kGearRatio2
                 ));

  // Initialize
  std::cout << "Initializing..." << std::endl;
  huron.Initialize();
  std::cout << "Initialization completed." << std::endl;

  std::cout << "Enabling motor..." << std::endl;
  huron.SetUp();
  std::cout << "Motor enabled." << std::endl;

  std::cout << "Initial position: \n";
  PrintVector<double>(huron.GetJointPosition());

  std::cout << "Moving..." << std::endl;
  auto start_time = std::chrono::steady_clock::now();

  huron.Move({0.2, 0.2, 0.2, 0.2});

  while (since(start_time).count() < 3000 /* ms */) {
    std::cout << "Current position: \n";
    PrintVector<double>(huron.GetJointPosition());

    std::cout << "Current velocity: \n";
    PrintVector<double>(huron.GetJointVelocity());
  }

  std::cout << "Stopping..." << std::endl;
  huron.Stop();

  std::cout << "Final position: \n";
  PrintVector<double>(huron.GetJointPosition());

  std::this_thread::sleep_for(std::chrono::seconds(5));

  huron.Terminate();
  std::cout << "Terminated." << std::endl;
}

