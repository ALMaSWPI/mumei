#include "CanDriver.hpp"
#include "huron_driver/can/huron_canbus.h"


// Send a CAN message on the bus
bool HURONCanBus::send_message(const can_Message_t &tx_msg) {

	struct can_frame raw_frame;
	raw_frame.can_id = tx_msg.id;
	raw_frame.can_dlc = tx_msg.len;
	memcpy(raw_frame.data, tx_msg.buf, tx_msg.len);
	
	sockcanpp::CanMessage msg_to_send(raw_frame);

	auto sent_byte_count = can_driver_.sendMessage(msg_to_send);
	
	return true;
}

bool HURONCanBus::recv_message(can_Message_t& message) {
	if (can_driver_.waitForMessages(recv_timeout_)) {
		// read a single message
		sockcanpp::CanMessage rx_msg = can_driver_.readMessage();
		// convert and save into [message]
		message.id = uint32_t(rx_msg.getCanId());
		message.isExt = rx_msg.getCanId().isExtendedFrameId();
		message.rtr = rx_msg.getCanId().hasRtrFrameFlag();
		message.len = rx_msg.getRawFrame().can_dlc;
		memcpy(message.buf, rx_msg.getRawFrame().data, message.len); 
		return true;
	}
	return false;
}

bool HURONCanBus::subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) {
	return false;
}

bool HURONCanBus::unsubscribe(CanSubscription* handle) {
	return false;
}

