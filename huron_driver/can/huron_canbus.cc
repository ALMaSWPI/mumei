#include "huron_canbus.h"


// Send a CAN message on the bus
bool HURONCanBus::send_message(const can_Message_t &txmsg) {
		if (HAL_CAN_GetError(handle_) != HAL_CAN_ERROR_NONE) {
				return false;
		}

		CAN_TxHeaderTypeDef header;
		header.StdId = txmsg.id;
		header.ExtId = txmsg.id;
		header.IDE = txmsg.isExt ? CAN_ID_EXT : CAN_ID_STD;
		header.RTR = CAN_RTR_DATA;
		header.DLC = txmsg.len;
		header.TransmitGlobalTime = FunctionalState::DISABLE;

		uint32_t retTxMailbox = 0;
		if (!HAL_CAN_GetTxMailboxesFreeLevel(handle_)) {
				return false;
		}
		
		return HAL_CAN_AddTxMessage(handle_, &header, (uint8_t*)txmsg.buf, &retTxMailbox) == HAL_OK;
}

//void HURONCanBus::set_error(Error error) {
//		error_ |= error;
//}

bool HURONCanBus::subscribe(const MsgIdFilterSpecs& filter, on_can_message_cb_t callback, void* ctx, CanSubscription** handle) {
		auto it = std::find_if(subscriptions_.begin(), subscriptions_.end(), [](auto& subscription) {
				return subscription.fifo == kCanFifoNone;
		});

		if (it == subscriptions_.end()) {
				return false; // all subscription slots in use
		}

		it->callback = callback;
		it->ctx = ctx;
		it->fifo = CAN_RX_FIFO0; // TODO: make customizable
		if (handle) {
				*handle = &*it;
		}

		bool is_extended = filter.id.index() == 1;
		uint32_t id = is_extended ?
									((std::get<1>(filter.id) << 3) | (1 << 2)) :
									(std::get<0>(filter.id) << 21);
		uint32_t mask = (is_extended ? (filter.mask << 3) : (filter.mask << 21))
									| (1 << 2); // care about the is_extended bit

		CAN_FilterTypeDef hal_filter;
		hal_filter.FilterActivation = ENABLE;
		hal_filter.FilterBank = &*it - &subscriptions_[0];
		hal_filter.FilterFIFOAssignment = it->fifo;
		hal_filter.FilterIdHigh = (id >> 16) & 0xffff;
		hal_filter.FilterIdLow = id & 0xffff;
		hal_filter.FilterMaskIdHigh = (mask >> 16) & 0xffff;
		hal_filter.FilterMaskIdLow = mask & 0xffff;
		hal_filter.FilterMode = CAN_FILTERMODE_IDMASK;
		hal_filter.FilterScale = CAN_FILTERSCALE_32BIT;

		if (HAL_CAN_ConfigFilter(handle_, &hal_filter) != HAL_OK) {
				return false;
		}
		return true;
}

bool HURONCanBus::unsubscribe(CanSubscription* handle) {
		ODriveCanSubscription* subscription = static_cast<ODriveCanSubscription*>(handle);
		if (subscription < subscriptions_.begin() || subscription >= subscriptions_.end()) {
				return false;
		}
		if (subscription->fifo != kCanFifoNone) {
				return false; // not in use
		}

		subscription->fifo = kCanFifoNone;

		CAN_FilterTypeDef hal_filter = {};
		hal_filter.FilterActivation = DISABLE;
		return HAL_CAN_ConfigFilter(handle_, &hal_filter) == HAL_OK;
}

