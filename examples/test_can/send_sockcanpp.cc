#include <sockcanpp/CanDriver.hpp>

void SendCanFrameExample() {
  sockcanpp::CanDriver canDriver{"can0", CAN_RAW};

  sockcanpp::CanMessage messageToSend(0, "8 bytes!");

  auto sentByteCount = canDriver.sendMessage(messageToSend);

  printf("Sent %d bytes via CAN!\n", sentByteCount);
}

int main(int argc, char* argv[]) {
  SendCanFrameExample();
  return 0;
}
