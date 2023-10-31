#include <chrono>
#include <iostream>
#include <memory>

#include <huron/driver/can/socket_can_bus.h>
#include <huron/odrive/odrive_can.h>
#include <huron/odrive/odrive_rotary_encoder.h>
#include <huron/utils/time.h>

const float kCPR = 4096.0;

int main(int argc, char* argv[]) {
  // TODO(dtbpkmte): make pointer to hcb unique_ptr
  huron::driver::can::SocketCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb, 0, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());
  huron::odrive::ODriveEncoder left_knee_encoder{kCPR,
    left_knee_odrive};
  auto start_time = std::chrono::steady_clock::now();
  while (since(start_time).count() < 3000 /* ms */) {
    std::cout << "RotaryEncoder count: "
              << left_knee_encoder.GetPosition() << std::endl;
  }
}
