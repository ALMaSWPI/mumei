#include <functional>
#include "huron/odrive/odrive_can.h"

namespace huron {
namespace odrive {

ODriveCAN::ODriveCAN(huron::driver::can::BusBase* canbus,
                     uint32_t axis_id,
                     std::unique_ptr<ODriveConfiguration> config,
                     uint32_t get_timeout)
  : ODrive(std::move(config), get_timeout),
    canbus_(canbus),
    axis_id_(axis_id) {}

void ODriveCAN::Initialize() {
}

void ODriveCAN::SetUp() {
}

void ODriveCAN::Terminate() {
}


bool ODriveCAN::GetMotorError(uint64_t& motor_error) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_MOTOR_ERROR;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  motor_error = can_getSignal<uint64_t>(msg, 0, 64, true);
    return true;
}

bool ODriveCAN::GetEncoderError(uint32_t& encoder_error) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_ENCODER_ERROR;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  encoder_error = can_getSignal<uint32_t>(msg, 0, 32, true);
  return true;
}

bool ODriveCAN::GetControllerError(uint32_t& controller_error) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_CONTROLLER_ERROR;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  controller_error = can_getSignal<uint32_t>(msg, 0, 32, true);
  return true;
}

bool ODriveCAN::GetSensorlessError(uint32_t& sensorless_error) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_SENSORLESS_ERROR;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  sensorless_error = can_getSignal<uint32_t>(msg, 0, 32, true);
  return true;
}

bool ODriveCAN::GetEncoderEstimates(float& pos, float& vel) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_ENCODER_ESTIMATES;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  // canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  pos = can_getSignal<float>(msg, 0, 32, true);
  vel = can_getSignal<float>(msg, 32, 32, true);
  return true;
}

bool ODriveCAN::GetEncoderCount(int32_t& shadow_cnt, int32_t& cnt_cpr) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_ENCODER_COUNT;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  shadow_cnt = can_getSignal<int32_t>(msg, 0, 32, true);
  cnt_cpr = can_getSignal<int32_t>(msg, 32, 32, true);
  return true;
}

bool ODriveCAN::GetIq(float& iq_setpoint, float& iq_measured) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_IQ;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  iq_setpoint = can_getSignal<float>(msg, 0, 32, true);
  iq_measured = can_getSignal<float>(msg, 32, 32, true);
  return true;
}

bool ODriveCAN::GetSensorlessEstimates(float& pos, float& vel) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_SENSORLESS_ESTIMATES;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  pos = can_getSignal<float>(msg, 0, 32, true);
  vel = can_getSignal<float>(msg, 32, 32, true);
  return true;
}

bool ODriveCAN::GetBusVoltageCurrent(float& bus_voltage, float& bus_current) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_BUS_VOLTAGE_CURRENT;
  msg.rtr = true;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  bus_voltage = can_getSignal<float>(msg, 0, 32, true);
  bus_current = can_getSignal<float>(msg, 32, 32, true);
  return true;
}

bool ODriveCAN::GetAdcVoltage(float& adc_voltage) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_GET_ADC_VOLTAGE;
  msg.rtr = false;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  canbus_->send_message(msg);
  if (!canbus_->recv_message(msg, get_timeout_)) {
    return false;
  }
  adc_voltage = can_getSignal<float>(msg, 0, 32, true);
  return true;
}

bool ODriveCAN::SetAxisNodeid(uint32_t axis_id) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_AXIS_NODE_ID;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<uint32_t>(msg, axis_id, 0, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetAxisRequestedState(uint32_t state) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_AXIS_REQUESTED_STATE;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<uint32_t>(msg, state, 0, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetAxisStartupConfig() {
  // Not Implemented
  return false;
}

bool ODriveCAN::SetInputPos(float input_pos, int16_t vel_ff,
                            int16_t torque_ff) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_INPUT_POS;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<float>(msg, input_pos, 0, 32, true);
  can_setSignal<int16_t>(msg, input_pos, 32, 16, true, 0.001, 0);
  can_setSignal<int16_t>(msg, input_pos, 48, 16, true, 0.001, 0);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetInputVel(float input_vel, float torque_ff) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_INPUT_VEL;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<float>(msg, input_vel, 0, 32, true);
  can_setSignal<float>(msg, torque_ff, 32, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetInputTorque(float input_torque) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_INPUT_TORQUE;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<float>(msg, input_torque, 0, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetControllerModes(int32_t control_mode,
                                   int32_t input_mode) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_CONTROLLER_MODES;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<int32_t>(msg, control_mode, 0, 32, true);
  can_setSignal<int32_t>(msg, input_mode, 32, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetLimits(float velocity_limit, float current_limit) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_LIMITS;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<float>(msg, velocity_limit, 0, 32, true);
  can_setSignal<float>(msg, current_limit, 32, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetTrajVelLimit(float traj_vel_limit) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_TRAJ_VEL_LIMIT;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<float>(msg, traj_vel_limit, 0, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetTrajAccelLimits(float traj_accel_limit,
                                   float traj_decel_limit) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_TRAJ_ACCEL_LIMITS;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<float>(msg, traj_accel_limit, 0, 32, true);
  can_setSignal<float>(msg, traj_decel_limit, 32, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetTrajInertia(float traj_inertia) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_TRAJ_INERTIA;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<float>(msg, traj_inertia, 0, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetLinearCount(int32_t position) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_LINEAR_COUNT;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<int32_t>(msg, position, 0, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetPosGain(float pos_gain) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_POS_GAIN;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<float>(msg, pos_gain, 0, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::SetVelGains(float vel_gain, float vel_interator_gain) {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_SET_VEL_GAINS;
  msg.isExt = false;
  msg.len = 8;
  can_setSignal<float>(msg, vel_gain, 0, 32, true);
  can_setSignal<float>(msg, vel_interator_gain, 32, 32, true);
  return canbus_->send_message(msg);
}

bool ODriveCAN::Nmt() {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_CO_NMT_CTRL;
  msg.isExt = false;
  msg.len = 8;
  return canbus_->send_message(msg);
}

bool ODriveCAN::Estop() {
  can_Message_t msg;
  msg.id = axis_id_ << NUM_CMD_ID_BITS;
  msg.id += MSG_ODRIVE_ESTOP;
  msg.isExt = false;
  msg.len = 8;
  // Sends message with RTR on
  return canbus_->send_message(msg);
}

bool ODriveCAN::ClearErrors() {
  return false;
}

bool ODriveCAN::StartAnticogging() {
  return false;
}

}  // namespace odrive
}  // namespace huron
