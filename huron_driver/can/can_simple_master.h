#ifndef __CAN_SIMPLE_MASTER_H_
#define __CAN_SIMPLE_MASTER_H_

#include "canbus.h"

#define AXIS_COUNT 1 // TODO: remove

class CANSimpleMaster {
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

		CANSimpleMaster(CanBusBase* canbus, uint32_t can_id, uint32_t axis_id)
			: canbus_(canbus), can_id_(can_id), axis_id_(axis_id) {}

		bool init();
		uint32_t ServiceStack();

	private:

		bool RenewSubscription(size_t i);
		bool SendHeartbeat();

		void handle_can_message(const can_Message_t& msg);

		void do_command(const can_Message_t& cmd);
		
		// Get functions (msg.rtr bit must be set)
		bool get_motor_error();
		bool get_encoder_error();
		bool get_controller_error();
		bool get_sensorless_error();
		bool get_encoder_estimates();
		bool get_encoder_count();
		bool get_iq();
		bool get_sensorless_estimates();
		bool get_bus_voltage_current();
		// msg.rtr bit must NOT be set
		bool get_adc_voltage(const can_Message_t& msg);

		// Set functions
		static void set_axis_nodeid(const can_Message_t& msg);
		static void set_axis_requested_state(const can_Message_t& msg);
		static void set_axis_startup_config(const can_Message_t& msg);
		static void set_input_pos(const can_Message_t& msg);
		static void set_input_vel(const can_Message_t& msg);
		static void set_input_torque(const can_Message_t& msg);
		static void set_controller_modes(const can_Message_t& msg);
		static void set_limits(const can_Message_t& msg);
		static void set_traj_vel_limit(const can_Message_t& msg);
		static void set_traj_accel_limits(const can_Message_t& msg);
		static void set_traj_inertia(const can_Message_t& msg);
		static void set_linear_count(const can_Message_t& msg);
		static void set_pos_gain(const can_Message_t& msg);
		static void set_vel_gains(const can_Message_t& msg);

		// Other functions
		static void nmt(const can_Message_t& msg);
		static void estop(const can_Message_t& msg);
		static void clear_errors(const can_Message_t& msg);
		static void start_anticogging(const can_Message_t& msg);

		static constexpr uint8_t NUM_NODE_ID_BITS = 6;
		static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

		// Utility functions
		static constexpr uint32_t get_node_id(uint32_t msgID) {
				return (msgID >> NUM_CMD_ID_BITS);	// Upper 6 or more bits
		};

		static constexpr uint8_t get_cmd_id(uint32_t msgID) {
				return (msgID & 0x01F);  // Bottom 5 bits
		}

		CanBusBase* canbus_;
		uint32_t can_id_;
		uint32_t axis_id_;
		CanBusBase::CanSubscription* subscription_handles_[AXIS_COUNT];

		// TODO: we this is a hack but actually we should use protocol hooks to
		// renew our filter when the node ID changes
		uint32_t node_ids_[AXIS_COUNT];
		bool extended_node_ids_[AXIS_COUNT];
};

#endif	// __CAN_SIMPLE_MASTER_H_
