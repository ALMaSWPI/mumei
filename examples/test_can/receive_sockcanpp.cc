#include <iostream>
#include <sockcanpp/CanDriver.hpp>

void receiveCanFramesExample() {
    sockcanpp::CanDriver canDriver("can1", CAN_RAW);

    if (canDriver.waitForMessages(sockcanpp::milliseconds(3000) /* timeout */)) {
        // read a single message
        sockcanpp::CanMessage receivedMessage = canDriver.readMessage();

        // read all available messages
        sockcanpp::queue<sockcanpp::CanMessage> receivedMessages = canDriver.readQueuedMessages();

        // handle CAN frames
        std::cout << receivedMessage.getFrameData() << std::endl;
    }
}

int main(int argc, char* argv[]) { 
    receiveCanFramesExample();
    return 0;
}
