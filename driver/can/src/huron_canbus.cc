#include "CanDriver.hpp"
#include "huron/driver/can/huron_canbus.h"
#include "huron/utils/time.h"


// Send a CAN message on the bus
bool HURONCanBus::send_message(const can_Message_t &tx_msg) {
  struct can_frame raw_frame;
  raw_frame.can_id = tx_msg.id;
  // Handle RTR bit for 11-bit ID
  if (tx_msg.rtr) {
    raw_frame.can_id |= 0x800;
  }
  raw_frame.can_dlc = tx_msg.len;
  std::memcpy(raw_frame.data, tx_msg.buf, tx_msg.len);
  sockcanpp::CanMessage msg_to_send(raw_frame);
  auto sent_byte_count = can_driver_.sendMessage(msg_to_send);
  return sent_byte_count;
}

bool HURONCanBus::recv_message(can_Message_t& message, uint32_t timeout) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    if (can_driver_.waitForMessages(recv_timeout_)) {
      // read a single message
      sockcanpp::CanMessage rx_msg = can_driver_.readMessage();
      uint32_t msg_can_id = uint32_t(rx_msg.getCanId());
      if (msg_can_id == message.id) {
        // convert and save into [message]
        message.id = msg_can_id;
        message.isExt = rx_msg.getCanId().isExtendedFrameId();
        message.rtr = rx_msg.getCanId().hasRtrFrameFlag();
        message.len = rx_msg.getRawFrame().can_dlc;
        std::memcpy(message.buf, rx_msg.getRawFrame().data, message.len);
        return true;
      }
    }
    if (since(start).count() > timeout)
      break;
  }
  return false;
}

bool HURONCanBus::subscribe(const MsgIdFilterSpecs& filter,
                            on_can_message_cb_t callback,
                            void* ctx, CanSubscription** handle) {
  return false;
}

bool HURONCanBus::unsubscribe(CanSubscription* handle) {
  return false;
}

