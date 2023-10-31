#pragma once

#include <string>
#include <sockcanpp/CanDriver.hpp>

#include "canbus.h"

#define CAN_CLK_HZ (16000000)
#define CAN_CLK_MHZ (16)

namespace huron {
namespace driver {
namespace can {

// Anonymous enum for defining the most common CAN baud rates
enum {
  CAN_BAUD_125K = 125000,
  CAN_BAUD_250K = 250000,
  CAN_BAUD_500K = 500000,
  CAN_BAUD_1000K = 1000000,
  CAN_BAUD_1M = 1000000
};

class SocketCanBus : public BusBase {
 public:
  static constexpr sockcanpp::milliseconds kRecvTimeout{1000};
  // struct Config_t {
  // 	uint32_t baud_rate = CAN_BAUD_250K;
  // 	Protocol protocol = PROTOCOL_SIMPLE;
  //
  // 	Bus* parent = nullptr; // set in apply_config()
  // 	void set_baud_rate(uint32_t value) { parent->set_baud_rate(value); }
  // };

  SocketCanBus(std::string can_if, uint32_t axis_id)
    : can_if_(can_if), axis_id_(axis_id), recv_timeout_(kRecvTimeout) {}
  SocketCanBus(const SocketCanBus&) = delete;
  SocketCanBus& operator=(const SocketCanBus&) = delete;
  ~SocketCanBus() = default;

  std::string can_if_;
  uint32_t axis_id_;
  sockcanpp::milliseconds recv_timeout_;
  sockcanpp::CanDriver can_driver_{can_if_, CAN_RAW};

  static const uint8_t kCanFifoNone = 0xff;

  bool send_message(const can_Message_t& message) final;
  bool recv_message(can_Message_t& message,
		    uint32_t timeout = UINT32_MAX) final;
  bool subscribe(const MsgIdFilterSpecs& filter,
		 on_can_message_cb_t callback, void* ctx,
		 CanSubscription** handle) final;
  bool unsubscribe(CanSubscription* handle) final;
};

}  // namespace can
}  // namespace driver
}  // namespace huron
