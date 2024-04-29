#include <iostream>

#include "mumei/driver/can/socket_can_bus.h"
#include "mumei/odrive/odrive_can.h"

int main(int argc, char* argv[]) {
  mumei::driver::can::SocketCanBus hcb{"can0", 0};
  mumei::odrive::ODriveCAN hoc{&hcb, 0,
    std::make_unique<mumei::odrive::ODrive::ODriveConfiguration>()};
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

