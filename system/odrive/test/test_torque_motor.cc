#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "huron/driver/can/huron_canbus.h"
#include "huron/odrive/torque_motor.h"

using namespace std::chrono_literals;

int main(int argc, char* argv[]) { 
  // TODO: make pointer to hcb unique_ptr
  HURONCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<HuronODriveCAN>(
    &hcb, 0);
  huron::odrive::TorqueMotor left_knee_motor{left_knee_odrive};

  std::cout << "Initializing..." << std::endl;
  left_knee_motor.Initialize();
  std::cout << "Initialization completed." << std::endl;

  std::cout << "Enabling motor..." << std::endl;
  left_knee_motor.SetUp();
  std::cout << "Motor enabled." << std::endl;

  std::cout << "Moving..." << std::endl;
  left_knee_motor.Move(0.5);
  std::this_thread::sleep_for(3s);
  std::cout << "Stopping..." << std::endl;
  left_knee_motor.Stop();

  std::this_thread::sleep_for(5s);

  std::cout << "Moving..." << std::endl;
  left_knee_motor.Move(-0.5);
  std::this_thread::sleep_for(3s);
  std::cout << "Stopping..." << std::endl;
  left_knee_motor.Stop();
  std::this_thread::sleep_for(2s);

  left_knee_motor.Terminate();
  std::cout << "Terminated." << std::endl;

}
