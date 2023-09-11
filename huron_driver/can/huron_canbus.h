#ifndef __HURON_CANBUS_H_
#define __HURON_CANBUS_H_

#include "sockcanpp/CanDriver.hpp"
#include "canbus.h"

#define CAN_CLK_HZ (16000000)
#define CAN_CLK_MHZ (16)

// Anonymous enum for defining the most common CAN baud rates
enum {
	CAN_BAUD_125K = 125000,
	CAN_BAUD_250K = 250000,
	CAN_BAUD_500K = 500000,
	CAN_BAUD_1000K = 1000000,
	CAN_BAUD_1M = 1000000
};

class HURONCanBus : public CanBusBase {
public:
	// struct Config_t {
	// 	uint32_t baud_rate = CAN_BAUD_250K;
	// 	Protocol protocol = PROTOCOL_SIMPLE;
	//
	// 	HURONCanBus* parent = nullptr; // set in apply_config()
	// 	void set_baud_rate(uint32_t value) { parent->set_baud_rate(value); }
	// };

	HURONCanBus(std::string can_id, uint32_t axis_id)
		: can_id_(can_id), axis_id_(axis_id) {}

	std::string can_id_;
	uint32_t axis_id_;
	sockcanpp::CanDriver can_driver_{can_id_, CAN_RAW};


private:
		static const uint8_t kCanFifoNone = 0xff;

		struct ODriveCanSubscription : CanSubscription {
				uint8_t fifo = kCanFifoNone;
				on_can_message_cb_t callback;
				void* ctx;
		};

		bool reinit();
		void can_server_thread();
		bool set_baud_rate(uint32_t baud_rate);
		void process_rx_fifo(uint32_t fifo);
		bool send_message(const can_Message_t& message) final;
		bool subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) final;
		bool unsubscribe(CanSubscription* handle) final;

};

#endif	// __HURON_CANBUS_H_
