#include <iostream>
#include "CanDriver.hpp"

void ReceiveCanFramesExample() {
  sockcanpp::CanDriver canDriver("can1", CAN_RAW);

  if (canDriver.waitForMessages(sockcanpp::milliseconds(3000))) {
    // read a single message
    sockcanpp::CanMessage receivedMessage = canDriver.readMessage();

    // handle CAN frames
    std::cout << "Message data:" << std::endl;
    std::cout << "\tCAN ID: "
              << uint32_t(receivedMessage.getCanId()) << std::endl;
    std::cout <<  "\tFrame Data: "
              << receivedMessage.getFrameData() << std::endl;
  }
}

int main(int argc, char* argv[]) {
  ReceiveCanFramesExample();
  return 0;
}
