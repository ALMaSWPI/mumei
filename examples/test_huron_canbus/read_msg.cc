#include <iostream>
#include <huron_driver/can/huron_canbus.h>

// Source: https://stackoverflow.com/questions/2808398/easily-measure-elapsed-time
template <
    class result_t   = std::chrono::milliseconds,
    class clock_t    = std::chrono::steady_clock,
    class duration_t = std::chrono::milliseconds
>
auto since(std::chrono::time_point<clock_t, duration_t> const& start)
{
    return std::chrono::duration_cast<result_t>(clock_t::now() - start);
}

void ReadMessage(uint32_t target_id) {
  HURONCanBus hcb{"can0", 0};
  std::cout << "HURONCanBus initialized successfully. Waiting for message...\n";

  // Measure time for reading a message for 3 trials
  uint32_t n_trials = 5;
  uint32_t sum_latency = 0;
  auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < n_trials; ++i) {
    std::cout << "Trial #" << i << std::endl;
    while (1) {
      can_Message_t msg;
      hcb.recv_message(msg);
      if (msg.id == target_id) {
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
  ReadMessage(9);
  return 0;
}
