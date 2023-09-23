#include <iostream>
#include <memory>

#include "huron/driver/can/huron_canbus.h"
#include "huron/odrive/encoder.h"

const float kCPR = 4096.0;

int main(int argc, char* argv[]) { 
  // TODO: make pointer to hcb unique_ptr
  HURONCanBus hcb{"can0", 0};
  auto left_knee_odrive = std::make_shared<HuronODriveCAN>(
    &hcb, 0);
  huron::odrive::Encoder left_knee_encoder{kCPR,
    left_knee_odrive};
  std::cout << "Encoder count: "
            << left_knee_encoder.GetCount() << std::endl;
}
