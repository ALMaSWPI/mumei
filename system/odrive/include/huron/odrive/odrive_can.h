#pragma once

/**
* See https://docs.odriverobotics.com/v/0.5.6/can-protocol.html for more information about this CAN API.
* */

#include <memory>
#include <string>

#include "odrive.h"
#include "huron/driver/can/canbus.h"

namespace huron {
namespace odrive {

class ODriveCAN : public ODrive {
 private:
  static const uint32_t kRecvTimeout = 100;  // ms

 public:
  enum {
    MSG_CO_NMT_CTRL = 0x000,  // CANOpen NMT Message REC
    MSG_ODRIVE_HEARTBEAT,
    MSG_ODRIVE_ESTOP,
    MSG_GET_MOTOR_ERROR,  // Errors
    MSG_GET_ENCODER_ERROR,
    MSG_GET_SENSORLESS_ERROR,
    MSG_SET_AXIS_NODE_ID,
    MSG_SET_AXIS_REQUESTED_STATE,
    MSG_SET_AXIS_STARTUP_CONFIG,
    MSG_GET_ENCODER_ESTIMATES,
    MSG_GET_ENCODER_COUNT,
    MSG_SET_CONTROLLER_MODES,
    MSG_SET_INPUT_POS,
    MSG_SET_INPUT_VEL,
    MSG_SET_INPUT_TORQUE,
    MSG_SET_LIMITS,
    MSG_START_ANTICOGGING,
    MSG_SET_TRAJ_VEL_LIMIT,
    MSG_SET_TRAJ_ACCEL_LIMITS,
    MSG_SET_TRAJ_INERTIA,
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,
    MSG_RESET_ODRIVE,
    MSG_GET_BUS_VOLTAGE_CURRENT,
    MSG_CLEAR_ERRORS,
    MSG_SET_LINEAR_COUNT,
    MSG_SET_POS_GAIN,
    MSG_SET_VEL_GAINS,
    MSG_GET_ADC_VOLTAGE,
    MSG_GET_CONTROLLER_ERROR,
    MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
  };
  /**
   * Constructor of ODriveCAN. As the CAN interface of ODrive v3.6 does not
   * allow reading configuration from hardware, a default configuration matrix
   * must be passed to the constructor.
   *
   * @pre The configuration is the same as on hardware component.
   */
  ODriveCAN(huron::driver::can::BusBase* canbus,
	    uint32_t axis_id,
	    std::unique_ptr<ODriveConfiguration> config,
	    uint32_t get_timeout = kGetTimeout);
  ODriveCAN(const ODriveCAN&) = delete;
  ODriveCAN& operator=(const ODriveCAN&) = delete;
  virtual ~ODriveCAN() = default;

  // GenericComponent interface
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  void ConfigureKey(std::string config_key, std::any config_value) override;

  // Get functions (msg.rtr bit must be set)
  bool GetMotorError(uint64_t& motor_error) override;
  bool GetEncoderError(uint32_t& encoder_error) override;
  bool GetControllerError(uint32_t& controller_error) override;
  bool GetSensorlessError(uint32_t& sensorless_error) override;
  bool GetEncoderEstimates(float& pos, float& vel) override;
  bool GetEncoderCount(int32_t& shadow_cnt, int32_t& cnt_cpr) override;
  bool GetIq(float& iq_setpoint, float& iq_measured) override;
  bool GetSensorlessEstimates(float& pos, float& vel) override;
  bool GetBusVoltageCurrent(float& bus_voltage, float& bus_current) override;
  // msg.rtr bit must NOT be set
  bool GetAdcVoltage(float& adc_voltage) override;

  // Set functions
  bool SetAxisNodeid(uint32_t axis_id) override;
  bool SetAxisRequestedState(uint32_t state) override;
  bool SetAxisStartupConfig() override;
  bool SetInputPos(float input_pos, int16_t vel_ff,
		   int16_t torque_ff) override;
  bool SetInputVel(float input_vel, float torque_ff) override;
  bool SetInputTorque(float input_torque) override;
  bool SetControllerModes(int32_t control_mode, int32_t input_mode) override;
  bool SetLimits(float velocity_limit, float current_limit) override;
  bool SetTrajVelLimit(float traj_vel_limit) override;
  bool SetTrajAccelLimits(float traj_accel_limit,
			  float traj_decel_limit) override;
  bool SetTrajInertia(float traj_inertia) override;
  bool SetLinearCount(int32_t position) override;
  bool SetPosGain(float pos_gain) override;
  bool SetVelGains(float vel_gain, float vel_interator_gain) override;

  // Other functions
  bool Nmt() override;
  bool Estop() override;
  bool ClearErrors() override;
  bool StartAnticogging() override;

  static constexpr uint8_t NUM_NODE_ID_BITS = 6;
  static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

  // Utility functions
  static constexpr uint32_t GetNodeId(uint32_t msgID) {
    return (msgID >> NUM_CMD_ID_BITS);  // Upper 6 or more bits
  }

  static constexpr uint8_t GetCmdId(uint32_t msgID) {
    return (msgID & 0x01F);  // Bottom 5 bits
  }

 private:
  huron::driver::can::BusBase* canbus_;
  uint32_t can_id_;
  uint32_t axis_id_;
  bool is_ext_ = false;
};

}  // namespace odrive
}  // namespace huron
