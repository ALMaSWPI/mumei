#include <iostream>

#include "huron/driver/can/socket_can_bus.h"
#include "huron/odrive/odrive_can.h"

int main(int argc, char* argv[]) {
  huron::driver::can::SocketCanBus hcb{"can0", 0};
  huron::odrive::ODriveCAN hoc{&hcb, 0};
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

