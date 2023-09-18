#ifndef __HURON_ODRIVE_CAN_H_
#define __HURON_ODRIVE_CAN_H_

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
		uint32_t ServiceStack();

	// private:

		void HandleCanMessage(const can_Message_t& msg);

		void DoCommand(const can_Message_t& cmd);
		
		// Get functions (msg.rtr bit must be set)
		bool GetMotorError();
		bool GetEncoderError();
		bool GetControllerError();
		bool GetSensorlessError();
		bool GetEncoderEstimates();
		bool GetEncoderCount();
		bool GetIq();
		bool GetSensorlessEstimates();
		bool GetBusVoltageCurrent();
		// msg.rtr bit must NOT be set
		bool GetAdcVoltage(const can_Message_t& msg);

		// Set functions
		static void SetAxisNodeid(const can_Message_t& msg);
		static void SetAxisRequestedState(const can_Message_t& msg);
		static void SetAxisStartupConfig(const can_Message_t& msg);
		static void SetInputPos(const can_Message_t& msg);
		static void SetInputVel(const can_Message_t& msg);
		static void SetInputTorque(const can_Message_t& msg);
		static void SetControllerModes(const can_Message_t& msg);
		static void SetLimits(const can_Message_t& msg);
		static void SetTrajVelLimit(const can_Message_t& msg);
		static void SetTrajAccelLimits(const can_Message_t& msg);
		static void SetTrajInertia(const can_Message_t& msg);
		static void SetLinearCount(const can_Message_t& msg);
		static void SetPosGain(const can_Message_t& msg);
		static void SetVelGains(const can_Message_t& msg);

		// Other functions
		static void Nmt(const can_Message_t& msg);
		static void Estop(const can_Message_t& msg);
		static void ClearErrors(const can_Message_t& msg);
		static void StartAnticogging(const can_Message_t& msg);

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

#endif	// __HURON_ODRIVE_CAN_H_
