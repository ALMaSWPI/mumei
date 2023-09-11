#include "huron_odrive_can.h"

#include <functional>

bool HuronODriveCAN::init() {
	return true;
}

void HuronODriveCAN::HandleCanMessage(const can_Message_t& msg) {
	//		 Frame
	// nodeID | CMD
	// 6 bits | 5 bits
	uint32_t nodeID = GetNodeId(msg.id);

	for (auto& axis : axes) {
		if ((axis.config_.can.node_id == nodeID) && (axis.config_.can.is_extended == msg.isExt)) {
			DoCommand(axis, msg);
			return;
		}
	}
}

void HuronODriveCAN::Nmt(const can_Message_t& msg) {
	// Not implemented
}

void HuronODriveCAN::Estop(const can_Message_t& msg) {
	// TODO
}

bool HuronODriveCAN::GetMotorError() {
	can_Message_t txmsg;

	txmsg.id = axis_id_;

	txmsg.id = node_id_ << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_MOTOR_ERROR;	// heartbeat ID
	txmsg.isExt = false;
	txmsg.len = 8;

	can_setSignal(txmsg, axis.motor_.error_, 0, 64, true);

	return canbus_->send_message(txmsg);
}

bool HuronODriveCAN::GetEncoderError() {
	can_Message_t txmsg;
	txmsg.id = axis_id_ << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_ENCODER_ERROR;	// heartbeat ID
	txmsg.isExt = is_ext_;
	txmsg.len = 8;

	can_setSignal(txmsg, axis.encoder_.error_, 0, 32, true);

	return canbus_->send_message(txmsg);
}

bool HuronODriveCAN::GetSensorlessError() {
	can_Message_t txmsg;
	txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_SENSORLESS_ERROR;  // heartbeat ID
	txmsg.isExt = axis.config_.can.is_extended;
	txmsg.len = 8;

	can_setSignal(txmsg, axis.sensorless_estimator_.error_, 0, 32, true);

	return canbus_->send_message(txmsg);
}

bool HuronODriveCAN::GetControllerError() {
	can_Message_t txmsg;
	txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_CONTROLLER_ERROR;  // heartbeat ID
	txmsg.isExt = axis.config_.can.is_extended;
	txmsg.len = 8;

	can_setSignal(txmsg, axis.controller_.error_, 0, 32, true);

	return canbus_->send_message(txmsg);
}

void HuronODriveCAN::SetAxisNodeid(const can_Message_t& msg) {
	axis.config_.can.node_id = can_getSignal<uint32_t>(msg, 0, 32, true);
}

void HuronODriveCAN::SetAxisRequestedState(const can_Message_t& msg) {
	axis.requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 32, true));
}

void HuronODriveCAN::SetAxisStartupConfig(const can_Message_t& msg) {
// Not Implemented
}

bool HuronODriveCAN::GetEncoderEstimates() {
	can_Message_t txmsg;
	txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_ENCODER_ESTIMATES;	// heartbeat ID
	txmsg.isExt = axis.config_.can.is_extended;
	txmsg.len = 8;

	can_setSignal<float>(txmsg, axis.controller_.pos_estimate_linear_src_.any().value_or(0.0f), 0, 32, true);
	can_setSignal<float>(txmsg, axis.controller_.vel_estimate_src_.any().value_or(0.0f), 32, 32, true);

	return canbus_->send_message(txmsg);
}

bool HuronODriveCAN::GetSensorlessEstimates() {
	can_Message_t txmsg;
	txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_SENSORLESS_ESTIMATES;  // heartbeat ID
	txmsg.isExt = axis.config_.can.is_extended;
	txmsg.len = 8;

	static_assert(sizeof(float) == sizeof(axis.sensorless_estimator_.pll_pos_));

	can_setSignal<float>(txmsg, axis.sensorless_estimator_.pll_pos_, 0, 32, true);
	can_setSignal<float>(txmsg, axis.sensorless_estimator_.vel_estimate_.any().value_or(0.0f), 32, 32, true);

	return canbus_->send_message(txmsg);
}

bool HuronODriveCAN::GetEncoderCount() {
	can_Message_t txmsg;
	txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_ENCODER_COUNT;
	txmsg.isExt = axis.config_.can.is_extended;
	txmsg.len = 8;

	can_setSignal<int32_t>(txmsg, axis.encoder_.shadow_count_, 0, 32, true);
	can_setSignal<int32_t>(txmsg, axis.encoder_.count_in_cpr_, 32, 32, true);
	return canbus_->send_message(txmsg);
}

void HuronODriveCAN::SetInputPos(const can_Message_t& msg) {
	axis.controller_.set_input_pos_and_steps(can_getSignal<float>(msg, 0, 32, true));
	axis.controller_.input_vel_ = can_getSignal<int16_t>(msg, 32, 16, true, 0.001f, 0);
	axis.controller_.input_torque_ = can_getSignal<int16_t>(msg, 48, 16, true, 0.001f, 0);
	axis.controller_.input_pos_updated();
}

void HuronODriveCAN::SetInputVel(const can_Message_t& msg) {
	axis.controller_.input_vel_ = can_getSignal<float>(msg, 0, 32, true);
	axis.controller_.input_torque_ = can_getSignal<float>(msg, 32, 32, true);
}

void HuronODriveCAN::SetInputTorque(const can_Message_t& msg) {
axis.controller_.input_torque_ = can_getSignal<float>(msg, 0, 32, true);
}

void HuronODriveCAN::SetControllerModes(const can_Message_t& msg) {
Controller::ControlMode const mode = static_cast<Controller::ControlMode>(can_getSignal<int32_t>(msg, 0, 32, true));
axis.controller_.config_.control_mode = static_cast<Controller::ControlMode>(mode);
axis.controller_.config_.input_mode = static_cast<Controller::InputMode>(can_getSignal<int32_t>(msg, 32, 32, true));
axis.controller_.control_mode_updated();
}

void HuronODriveCAN::SetLimits(const can_Message_t& msg) {
axis.controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
axis.motor_.config_.current_lim = can_getSignal<float>(msg, 32, 32, true);
}

void HuronODriveCAN::StartAnticogging(const can_Message_t& msg) {
axis.controller_.start_anticogging_calibration();
}

void HuronODriveCAN::SetTrajVelLimit(const can_Message_t& msg) {
axis.trap_traj_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void HuronODriveCAN::SetTrajAccelLimits(const can_Message_t& msg) {
	axis.trap_traj_.config_.accel_limit = can_getSignal<float>(msg, 0, 32, true);
	axis.trap_traj_.config_.decel_limit = can_getSignal<float>(msg, 32, 32, true);
}

void HuronODriveCAN::SetTrajInertia(const can_Message_t& msg) {
	axis.controller_.config_.inertia = can_getSignal<float>(msg, 0, 32, true);
}

void HuronODriveCAN::SetLinearCount(const can_Message_t& msg) {
	axis.encoder_.set_linear_count(can_getSignal<int32_t>(msg, 0, 32, true));
}

void HuronODriveCAN::SetPosGain(const can_Message_t& msg) {
	axis.controller_.config_.pos_gain = can_getSignal<float>(msg, 0, 32, true);
}

void HuronODriveCAN::SetVelGains(const can_Message_t& msg) {
	axis.controller_.config_.vel_gain = can_getSignal<float>(msg, 0, 32, true);
	axis.controller_.config_.vel_integrator_gain = can_getSignal<float>(msg, 32, 32, true);
}

bool HuronODriveCAN::GetIq() {
	can_Message_t txmsg;
	txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_IQ;
	txmsg.isExt = axis.config_.can.is_extended;
	txmsg.len = 8;

	std::optional<float2D> Idq_setpoint = axis.motor_.current_control_.Idq_setpoint_;
	if (!Idq_setpoint.has_value()) {
	Idq_setpoint = {0.0f, 0.0f};
}

static_assert(sizeof(float) == sizeof(Idq_setpoint->second));
static_assert(sizeof(float) == sizeof(axis.motor_.current_control_.Iq_measured_));
can_setSignal<float>(txmsg, Idq_setpoint->second, 0, 32, true);
can_setSignal<float>(txmsg, axis.motor_.current_control_.Iq_measured_, 32, 32, true);

return canbus_->send_message(txmsg);
}

bool HuronODriveCAN::GetBusVoltageCurrent() {
can_Message_t txmsg;

txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
txmsg.id += MSG_GET_BUS_VOLTAGE_CURRENT;
txmsg.isExt = axis.config_.can.is_extended;
txmsg.len = 8;

static_assert(sizeof(float) == sizeof(vbus_voltage));
static_assert(sizeof(float) == sizeof(ibus_));
can_setSignal<float>(txmsg, vbus_voltage, 0, 32, true);
can_setSignal<float>(txmsg, ibus_, 32, 32, true);

return canbus_->send_message(txmsg);
}

bool HuronODriveCAN::GetAdcVoltage(const can_Message_t& msg) {
	can_Message_t txmsg;

	txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_ADC_VOLTAGE;
	txmsg.isExt = axis.config_.can.is_extended;
	txmsg.len = 8;

	auto gpio_num = can_getSignal<uint8_t>(msg, 0, 8, true);
	if (gpio_num < GPIO_COUNT) {
		auto voltage = GetAdcVoltage(get_gpio(gpio_num));
		can_setSignal<float>(txmsg, voltage, 0, 32, true);
		return canbus_->send_message(txmsg);
	} else {
		return false;
	}
}

