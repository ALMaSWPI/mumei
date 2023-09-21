#pragma once

/**
* See https://docs.odriverobotics.com/v/0.5.6/can-protocol.html for more information about this CAN API.
* */

#include "canbus.h"

class HuronODriveCAN {
	 public:
		enum {
				MSG_CO_NMT_CTRL = 0x000,	// CANOpen NMT Message REC
				MSG_ODRIVE_HEARTBEAT,
				MSG_ODRIVE_ESTOP,
				MSG_GET_MOTOR_ERROR,	// Errors
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

		HuronODriveCAN(CanBusBase* canbus, uint32_t axis_id)
			: canbus_(canbus), axis_id_(axis_id) {}

		bool init();

	// private:

		// Get functions (msg.rtr bit must be set)
		bool GetMotorError(uint64_t& motor_error, uint32_t timeout = 0);
		bool GetEncoderError(uint32_t& encoder_error, uint32_t timeout = 0);
		bool GetControllerError(uint32_t& controller_error, uint32_t timeout = 0);
		bool GetSensorlessError(uint32_t& sensorless_error, uint32_t timeout = 0);
		bool GetEncoderEstimates(float& pos, float& vel, uint32_t timeout = 0);
		bool GetEncoderCount(int32_t& shadow_cnt, int32_t& cnt_cpr,
											 uint32_t timeout = 0);
		bool GetIq(float& iq_setpoint, float& iq_measured, uint32_t timeout = 0);
		bool GetSensorlessEstimates(float& pos, float& vel, uint32_t timeout = 0);
		bool GetBusVoltageCurrent(float& bus_voltage, float& bus_current,
														uint32_t timeout = 0);
		// msg.rtr bit must NOT be set
		bool GetAdcVoltage(float& adc_voltage, uint32_t timeout = 0);

		// Set functions
		bool SetAxisNodeid(uint32_t axis_id);
		bool SetAxisRequestedState(uint32_t state);
		bool SetAxisStartupConfig();
		bool SetInputPos(float input_pos, int16_t vel_ff, int16_t torque_ff);
		bool SetInputVel(float input_vel, float torque_ff);
		bool SetInputTorque(float input_torque);
		bool SetControllerModes(int32_t control_mode, int32_t input_mode);
		bool SetLimits(float velocity_limit, float current_limit);
		bool SetTrajVelLimit(float traj_vel_limit);
		bool SetTrajAccelLimits(float traj_accel_limit, float traj_decel_limit);
		bool SetTrajInertia(float traj_inertia);
		bool SetLinearCount(int32_t position);
		bool SetPosGain(float pos_gain);
		bool SetVelGains(float vel_gain, float vel_interator_gain);

		// Other functions
		bool Nmt();
		bool Estop();
		bool ClearErrors();
		bool StartAnticogging();

		static constexpr uint8_t NUM_NODE_ID_BITS = 6;
		static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

		// Utility functions
		static constexpr uint32_t GetNodeId(uint32_t msgID) {
				return (msgID >> NUM_CMD_ID_BITS);	// Upper 6 or more bits
		};

		static constexpr uint8_t GetCmdId(uint32_t msgID) {
				return (msgID & 0x01F);  // Bottom 5 bits
		}

		CanBusBase* canbus_;
		uint32_t can_id_;
		uint32_t axis_id_;
		bool is_ext_ = false;
};

