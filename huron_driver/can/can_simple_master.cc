#include "can_simple_master.h"

#include <functional>

bool CANSimpleMaster::init() {
	for (size_t i = 0; i < AXIS_COUNT; ++i) {
		if (!renew_subscription(i)) {
			return false;
	}
}

	return true;
}

void CANSimpleMaster::HandleCanMessage(const can_Message_t& msg) {
	//		 Frame
	// nodeID | CMD
	// 6 bits | 5 bits
	uint32_t nodeID = get_node_id(msg.id);

	for (auto& axis : axes) {
		if ((axis.config_.can.node_id == nodeID) && (axis.config_.can.is_extended == msg.isExt)) {
			do_command(axis, msg);
			return;
		}
	}
}

void CANSimpleMaster::nmt(const can_Message_t& msg) {
	// Not implemented
}

void CANSimpleMaster::estop(const can_Message_t& msg) {
	// TODO
}

bool CANSimpleMaster::GetMotorError() {
	can_Message_t txmsg;

	txmsg.id = this.axis_id_;

	txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
	txmsg.id += MSG_GET_MOTOR_ERROR;	// heartbeat ID
	txmsg.isExt = false;
	txmsg.len = 8;

	can_setSignal(txmsg, axis.motor_.error_, 0, 64, true);

	return canbus_->send_message(txmsg);
}

bool CANSimpleMaster::get_encoder_error() {
can_Message_t txmsg;
txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
txmsg.id += MSG_GET_ENCODER_ERROR;	// heartbeat ID
txmsg.isExt = axis.config_.can.is_extended;
txmsg.len = 8;

can_setSignal(txmsg, axis.encoder_.error_, 0, 32, true);

return canbus_->send_message(txmsg);
}

bool CANSimpleMaster::get_sensorless_error() {
can_Message_t txmsg;
txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
txmsg.id += MSG_GET_SENSORLESS_ERROR;  // heartbeat ID
txmsg.isExt = axis.config_.can.is_extended;
txmsg.len = 8;

can_setSignal(txmsg, axis.sensorless_estimator_.error_, 0, 32, true);

return canbus_->send_message(txmsg);
}

bool CANSimpleMaster::get_controller_error() {
can_Message_t txmsg;
txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
txmsg.id += MSG_GET_CONTROLLER_ERROR;  // heartbeat ID
txmsg.isExt = axis.config_.can.is_extended;
txmsg.len = 8;

can_setSignal(txmsg, axis.controller_.error_, 0, 32, true);

return canbus_->send_message(txmsg);
}

void CANSimpleMaster::set_axis_nodeid(const can_Message_t& msg) {
axis.config_.can.node_id = can_getSignal<uint32_t>(msg, 0, 32, true);
}

void CANSimpleMaster::set_axis_requested_state(const can_Message_t& msg) {
axis.requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 32, true));
}

void CANSimpleMaster::set_axis_startup_config(const can_Message_t& msg) {
// Not Implemented
}

bool CANSimpleMaster::get_encoder_estimates() {
can_Message_t txmsg;
txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
txmsg.id += MSG_GET_ENCODER_ESTIMATES;	// heartbeat ID
txmsg.isExt = axis.config_.can.is_extended;
txmsg.len = 8;

can_setSignal<float>(txmsg, axis.controller_.pos_estimate_linear_src_.any().value_or(0.0f), 0, 32, true);
can_setSignal<float>(txmsg, axis.controller_.vel_estimate_src_.any().value_or(0.0f), 32, 32, true);

return canbus_->send_message(txmsg);
}

bool CANSimpleMaster::get_sensorless_estimates() {
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

bool CANSimpleMaster::get_encoder_count() {
can_Message_t txmsg;
txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
txmsg.id += MSG_GET_ENCODER_COUNT;
txmsg.isExt = axis.config_.can.is_extended;
txmsg.len = 8;

can_setSignal<int32_t>(txmsg, axis.encoder_.shadow_count_, 0, 32, true);
can_setSignal<int32_t>(txmsg, axis.encoder_.count_in_cpr_, 32, 32, true);
return canbus_->send_message(txmsg);
}

void CANSimpleMaster::set_input_pos(const can_Message_t& msg) {
axis.controller_.set_input_pos_and_steps(can_getSignal<float>(msg, 0, 32, true));
axis.controller_.input_vel_ = can_getSignal<int16_t>(msg, 32, 16, true, 0.001f, 0);
axis.controller_.input_torque_ = can_getSignal<int16_t>(msg, 48, 16, true, 0.001f, 0);
axis.controller_.input_pos_updated();
}

void CANSimpleMaster::set_input_vel(const can_Message_t& msg) {
axis.controller_.input_vel_ = can_getSignal<float>(msg, 0, 32, true);
axis.controller_.input_torque_ = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimpleMaster::set_input_torque(const can_Message_t& msg) {
axis.controller_.input_torque_ = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimpleMaster::set_controller_modes(const can_Message_t& msg) {
Controller::ControlMode const mode = static_cast<Controller::ControlMode>(can_getSignal<int32_t>(msg, 0, 32, true));
axis.controller_.config_.control_mode = static_cast<Controller::ControlMode>(mode);
axis.controller_.config_.input_mode = static_cast<Controller::InputMode>(can_getSignal<int32_t>(msg, 32, 32, true));
axis.controller_.control_mode_updated();
}

void CANSimpleMaster::set_limits(const can_Message_t& msg) {
axis.controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
axis.motor_.config_.current_lim = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimpleMaster::start_anticogging(const can_Message_t& msg) {
axis.controller_.start_anticogging_calibration();
}

void CANSimpleMaster::set_traj_vel_limit(const can_Message_t& msg) {
axis.trap_traj_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimpleMaster::set_traj_accel_limits(const can_Message_t& msg) {
axis.trap_traj_.config_.accel_limit = can_getSignal<float>(msg, 0, 32, true);
axis.trap_traj_.config_.decel_limit = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimpleMaster::set_traj_inertia(const can_Message_t& msg) {
axis.controller_.config_.inertia = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimpleMaster::set_linear_count(const can_Message_t& msg) {
axis.encoder_.set_linear_count(can_getSignal<int32_t>(msg, 0, 32, true));
}

void CANSimpleMaster::set_pos_gain(const can_Message_t& msg) {
axis.controller_.config_.pos_gain = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimpleMaster::set_vel_gains(const can_Message_t& msg) {
axis.controller_.config_.vel_gain = can_getSignal<float>(msg, 0, 32, true);
axis.controller_.config_.vel_integrator_gain = can_getSignal<float>(msg, 32, 32, true);
}

bool CANSimpleMaster::get_iq() {
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

bool CANSimpleMaster::get_bus_voltage_current() {
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

bool CANSimpleMaster::get_adc_voltage(const can_Message_t& msg) {
can_Message_t txmsg;

txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
txmsg.id += MSG_GET_ADC_VOLTAGE;
txmsg.isExt = axis.config_.can.is_extended;
txmsg.len = 8;

auto gpio_num = can_getSignal<uint8_t>(msg, 0, 8, true);
if (gpio_num < GPIO_COUNT) {
auto voltage = get_adc_voltage(get_gpio(gpio_num));
can_setSignal<float>(txmsg, voltage, 0, 32, true);
return canbus_->send_message(txmsg);
} else {
return false;
}
}

