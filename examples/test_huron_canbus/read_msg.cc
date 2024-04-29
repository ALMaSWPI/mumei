#include <iostream>

#include "mumei/driver/can/socket_can_bus.h"
#include "mumei/utils/time.h"

void ReadMessage(can_Message_t& msg) {
  mumei::driver::can::SocketCanBus hcb{"can0", 0};
  std::cout << "HURONCanBus initialized successfully. Waiting for message...\n";

  // Measure time for reading a message for 3 trials
  uint32_t n_trials = 5;
  uint32_t sum_latency = 0;
  for (size_t i = 0; i < n_trials; ++i) {
    std::cout << "Trial #" << i << std::endl;
    auto start = std::chrono::steady_clock::now();
    while (1) {
      if (hcb.recv_message(msg)) {
        // handle CAN frames
        // msg val startBit length isIntel
        float pos = can_getSignal<float>(msg, 0, 32, true);
        float vel = can_getSignal<float>(msg, 32, 32, true);
        std::cout << "Message data:" << std::endl;
        std::cout << "\tCAN ID: "
                  << msg.id << std::endl;
        std::cout << "\tPos: " << pos
                  << "\tVel: " << vel << std::endl;
        uint32_t latency = since(start).count();
        std::cout << "Time: " << latency << " ms.\n\n";
        sum_latency += latency;
        break;
      }
    }
  }
  std::cout << "Average latency: " << float(sum_latency)/n_trials << std::endl;
}

int main(int argc, char* argv[]) {
  can_Message_t msg;
  msg.id = 9;
  ReadMessage(msg);
  return 0;
}
