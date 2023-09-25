#include <iostream>

#include "huron/driver/can/huron_odrive_can.h"
#include "huron/driver/can/socket_can_bus.h"

int main(int argc, char* argv[]) {
  huron::driver::can::SocketCanBus hcb{"can0", 0};
  huron::odrive::can::ODrive hoc{&hcb, 0};
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

