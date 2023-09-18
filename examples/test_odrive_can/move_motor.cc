#include <iostream>
#include <chrono>
#include <thread>
#include "huron/driver/can/huron_canbus.h"
#include "huron/driver/can/huron_odrive_can.h"
#include "huron/driver/can/ODriveEnums.h"

using namespace std::chrono_literals;


int main(int argc, char* argv[]) { 
  HURONCanBus hcb{"can0", 0};
  HuronODriveCAN hoc{&hcb, 0};

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

