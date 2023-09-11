#include "sockcanpp/CanDriver.hpp"
#include "huron_canbus.h"


// Send a CAN message on the bus
bool HURONCanBus::send_message(const can_Message_t &txmsg) {

	struct can_frame raw_frame;
	raw_frame.can_id = txmsg.id;
	raw_frame.len = txmsg.len;
	memcpy(raw_frame.data, txmsg.buf, txmsg.len);
	
	sockcanpp::CanMessage msg_to_send(raw_frame);

	auto sent_byte_count = can_driver_.sendMessage(msg_to_send);
	
	return true;
}

//void HURONCanBus::set_error(Error error) {
//		error_ |= error;
//}

bool HURONCanBus::subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) {
	return false;
}

bool HURONCanBus::unsubscribe(CanSubscription* handle) {
	return false;
}

