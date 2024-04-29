#include <iostream>
#include <chrono>
#include <thread>

#include "mumei/utils/time.h"
#include "mumei/driver/serial/wjwwood_serial.h"
#include "mumei/sensors/force_sensing_resistor_array_serial.h"

int main(int argc, char* argv[]) {
  mumei::ForceSensingResistorArraySerial fsr_array{
    "LFoot",
    4,
    std::make_shared<mumei::driver::serial::Serial>(
      "/dev/ttyACM0",
      115200,
      mumei::driver::serial::Parity::None,
      mumei::driver::serial::StopBits::One,
      mumei::driver::serial::FlowControl::None)};

  while (true) {
    std::cout << fsr_array.GetValues() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}
