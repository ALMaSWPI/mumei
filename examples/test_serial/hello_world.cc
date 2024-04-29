#include <iostream>
#include <chrono>
#include <thread>

#include "mumei/utils/time.h"
#include "mumei/driver/serial/wjwwood_serial.h"

int main(int argc, char* argv[]) {
  using namespace mumei::driver::serial; //NOLINT
  Serial serial("/dev/ttyACM0", 115200, Parity::None, StopBits::One,
                FlowControl::None);
  std::cout << "Serial port status: ";
  if (serial.IsOpen()) {
    std::cout << "Open" << std::endl;
  } else {
    std::cout << "Closed" << std::endl;
    return 0;
  }

  while (true) {
    std::cout << serial.ReadLine() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}
