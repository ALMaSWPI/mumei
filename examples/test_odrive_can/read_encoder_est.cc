#include <iostream>

#include "mumei/driver/can/socket_can_bus.h"
#include "mumei/odrive/odrive_can.h"

int main(int argc, char* argv[]) {
  mumei::driver::can::SocketCanBus hcb{"can0", 0};
  mumei::odrive::ODriveCAN hoc{&hcb, 0,
    std::make_unique<mumei::odrive::ODrive::ODriveConfiguration>()};
  float pos = 0.0, vel = 0.0;
  bool success = hoc.GetEncoderEstimates(pos, vel);
  if (!success)
    std::cout << "Get encoder estimates failed.\n";
  else
    std::cout << "Get encoder estimates succesful.\n";
  std::cout << "Pos: " << pos
            << "\tVel: " << vel << std::endl;
  return 0;
}

