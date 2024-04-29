#include <chrono>
#include <iostream>
#include <memory>

#include "mumei/driver/can/socket_can_bus.h"
#include "mumei/odrive/odrive_can.h"
#include "mumei/odrive/odrive_rotary_encoder.h"
#include "mumei/utils/time.h"

const float kCPR = 4096.0;

int main(int argc, char* argv[]) {
  // TODO(dtbpkmte): make pointer to hcb unique_ptr
  mumei::driver::can::SocketCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<mumei::odrive::ODriveCAN>(
    &hcb, 0, std::make_unique<mumei::odrive::ODrive::ODriveConfiguration>());
  mumei::odrive::ODriveEncoder left_knee_encoder{kCPR,
    left_knee_odrive};
  auto start_time = std::chrono::steady_clock::now();
  while (since(start_time).count() < 3000 /* ms */) {
    std::cout << "RotaryEncoder count: "
              << left_knee_encoder.GetPosition() << std::endl;
  }
}
