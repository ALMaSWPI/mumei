#include <sockcanpp/CanDriver.hpp>

void sendCanFrameExample() {
    sockcanpp::CanDriver canDriver{"can0", CAN_RAW};

    sockcanpp::CanMessage messageToSend(0 /*send with default ID*/, "8 bytes!" /* the data */);

    auto sentByteCount = canDriver.sendMessage(messageToSend);

    printf("Sent %d bytes via CAN!\n", sentByteCount);
}

int main(int argc, char* argv[]) { 
    sendCanFrameExample();
    return 0;
}
