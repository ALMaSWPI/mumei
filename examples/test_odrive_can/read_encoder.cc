#include <iostream>
#include "huron/driver/can/huron_canbus.h"
#include "huron/driver/can/huron_odrive_can.h"

int main(int argc, char* argv[]) {
  HURONCanBus hcb{"can0", 0};
  HuronODriveCAN hoc{&hcb, 0};
  int32_t shadow_cnt = 0, cnt_cpr = 0;
  bool success = hoc.GetEncoderCount(shadow_cnt, cnt_cpr);
  if (!success)
    std::cout << "Get encoder count failed.\n";
  else
    std::cout << "Get encoder count succesful.\n";
  std::cout << "Shadow count: " << shadow_cnt
            << "\tCount in CPR: " << cnt_cpr << std::endl;
  return 0;
}

