#include <iostream>
#include "huron/driver/can/huron_canbus.h"
#include "huron/driver/can/huron_odrive_can.h"


int main(int argc, char* argv[]) { 
  HURONCanBus hcb{"can0", 0};
  HuronODriveCAN hoc{&hcb, 0};
  float pos = 0.0, vel = 0.0;
  hoc.GetEncoderEstimates(pos, vel);
  std::cout << "Pos: " << pos
            << "\tVel: " << vel << std::endl;
  return 0;
}

