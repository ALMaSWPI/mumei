#include <iostream>
#include <huron_driver/can/huron_canbus.h>
#include <huron_driver/can/huron_odrive_can.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

bool SendMsg(uint32_t msg_id, float val) {
  HURONCanBus hcb{"can0", 0};
  std::cout << "HURONCanBus initialized successfully.\n";

  can_Message_t msg;
	msg.id = hcb.axis_id_ << HuronODriveCAN::NUM_CMD_ID_BITS;
	msg.id += msg_id;
	msg.isExt = false;
	msg.len = 8;
  can_setSignal<float>(msg, val, 0, 32, true);
  return hcb.send_message(msg);
}

int main(int argc, char* argv[]) { 
  SendMsg(HuronODriveCAN::MSG_SET_INPUT_TORQUE, 0.7);
  std::this_thread::sleep_for(2s);
  SendMsg(HuronODriveCAN::MSG_SET_INPUT_TORQUE, 0.0);
  return 0;
}
