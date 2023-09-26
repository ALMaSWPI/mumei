#include <chrono>//NOLINT
#include <iostream>
#include <thread>//NOLINT

#include "huron/driver/can/socket_can_bus.h"
#include "huron/odrive/odrive_can.h"
#include "huron/odrive/odrive_enums.h"

using namespace std::chrono_literals;  //NOLINT

int main(int argc, char* argv[]) {
  huron::driver::can::SocketCanBus hcb{"can0", 0};
  huron::odrive::ODriveCAN hoc{&hcb, 0};

  hoc.SetAxisRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL);

  std::this_thread::sleep_for(2s);

  bool success = hoc.SetInputTorque(-0.5);
  if (!success)
    std::cout << "Set torque failed.\n";
  else
    std::cout << "Set torque succesful.\n";

  std::this_thread::sleep_for(2s);

  hoc.SetInputTorque(0.0);

  hoc.SetAxisRequestedState(AXIS_STATE_IDLE);
  return 0;
}

