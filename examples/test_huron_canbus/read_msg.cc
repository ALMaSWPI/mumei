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
  sockcanpp::CanDriver canDriver("can0", CAN_RAW);
  std::cout << "CanDriver initialized successfully. Waiting for message...\n";

  // Measure time for reading a message
  auto start = std::chrono::steady_clock::now();
  while (1) {
    if (canDriver.waitForMessages(sockcanpp::milliseconds(3000))) {
      // read a single message
      sockcanpp::CanMessage receivedMessage = canDriver.readMessage();

      uint32_t id = uint32_t(receivedMessage.getCanId());
      if (id == target_id) {
        // handle CAN frames
        std::cout << "Message data:" << std::endl;
        std::cout << "\tCAN ID: "
                  << id << std::endl;
        std::cout <<  "\tFrame Data: " 
                  << receivedMessage.getFrameData() << std::endl;
        std::cout << "Time: " << since(start).count() << std::endl;
        break;
      }
    }
  }
}

int main(int argc, char* argv[]) { 
  ReadMessage(9);
  return 0;
}
