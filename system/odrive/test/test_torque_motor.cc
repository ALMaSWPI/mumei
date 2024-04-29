#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include "mumei/driver/can/socket_can_bus.h"
#include "mumei/odrive/odrive_torque_motor.h"

int main(int argc, char* argv[]) {
  // TODO(dtbpkmte): make pointer to hcb unique_ptr
  mumei::driver::can::SocketCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<mumei::odrive::ODriveCAN>(
    &hcb, 0, std::make_unique<mumei::odrive::ODrive::ODriveConfiguration>());
  mumei::odrive::TorqueMotor left_knee_motor{left_knee_odrive};

  std::cout << "Initializing..." << std::endl;
  left_knee_motor.Initialize();
  std::cout << "Initialization completed." << std::endl;

  std::cout << "Enabling motor..." << std::endl;
  left_knee_motor.SetUp();
  std::cout << "Motor enabled." << std::endl;

  std::cout << "Moving..." << std::endl;
  left_knee_motor.Move(0.5);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  std::cout << "Stopping..." << std::endl;
  left_knee_motor.Stop();

  std::this_thread::sleep_for(std::chrono::seconds(5));

  std::cout << "Moving..." << std::endl;
  left_knee_motor.Move(-0.5);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  std::cout << "Stopping..." << std::endl;
  left_knee_motor.Stop();
  std::this_thread::sleep_for(std::chrono::seconds(2));

  left_knee_motor.Terminate();
  std::cout << "Terminated." << std::endl;
}
