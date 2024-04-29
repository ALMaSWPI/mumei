#include <chrono>//NOLINT
#include <iostream>
#include <thread>//NOLINT

#include "mumei/driver/can/socket_can_bus.h"
#include "mumei/odrive/odrive_can.h"
#include "mumei/odrive/odrive_enums.h"

using namespace std::chrono_literals;  //NOLINT

int main(int argc, char* argv[]) {
  mumei::driver::can::SocketCanBus hcb{"can0", 0};
  mumei::odrive::ODriveCAN hoc{&hcb, 0,
    std::make_unique<mumei::odrive::ODrive::ODriveConfiguration>()};

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

