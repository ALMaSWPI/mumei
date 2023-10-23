#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <memory>

// For debug
#include <iostream>

#include "odrive_enums.h"
#include "huron/control_interfaces/generic_component.h"

namespace huron {
namespace odrive {

/**
 * Interface for using ODrive motor controllers.
 */
class ODrive : public huron::GenericComponent {
 protected:
  static const uint32_t kGetTimeout = 100;  // ms

  uint32_t get_timeout_;
  bool is_calibrated_ = false;

 public:
  class ODriveConfiguration : public huron::Configuration {
   public:
    /**
     * Supports further inheritance.
     */
    ODriveConfiguration(ConfigMap config_map,
			                  std::set<std::string> valid_keys)
      : huron::Configuration(config_map,
                             [&valid_keys]() {
                               std::set<std::string> tmp(kODriveKeys);
                               tmp.merge(valid_keys);
                               return tmp;
                             }()) {}

    explicit ODriveConfiguration(ConfigMap config_map)
      : ODriveConfiguration(config_map, std::set<std::string>()) {
      std::cout << "ODrive config:\n";
      for (auto& pair : config_map_) {
        std::cout << pair.first << " : " 
                  << std::any_cast<float>(pair.second) << std::endl;
      }
      std::cout << std::endl;
    }

    /**
     * Default constructor of ODriveConfiguration. This constructor is not
     * recommended as for some protocols like CAN, ODrive cannot read the
     * config values from hardware. The recommended way is to initialize the
     * local config map with initial values.
     */
    ODriveConfiguration()
      : ODriveConfiguration(ConfigMap(), std::set<std::string>()) {}

   private:
    static const inline std::set<std::string> kODriveKeys{
      "velocity_limit",
      "current_limit",
      "traj_vel_limit",
      "traj_accel_limit",
      "traj_decel_limit",
      "traj_inertia"};
  };
  ODrive(std::unique_ptr<ODriveConfiguration> config, uint32_t get_timeout)
      : huron::GenericComponent(std::move(config)),
	get_timeout_(get_timeout) {}
  explicit ODrive(uint32_t get_timeout = kGetTimeout)
      : ODrive(std::make_unique<ODriveConfiguration>(),
	       get_timeout) {}
  ODrive(const ODrive&) = delete;
  ODrive& operator=(const ODrive&) = delete;
  ~ODrive() override = default;

  /**
   * Puts the ODrive in IDLE state and, if not completed before, perform
   * full calibration.
   */
  void Initialize() override;

  /**
   * Performs full calibration of the ODrive.
   */
  bool Calibrate();

  void ConfigureKey(std::string config_key, std::any config_value) override;

  // Get functions (msg.rtr bit must be set)
  virtual bool GetMotorError(uint64_t& motor_error) = 0;
  virtual bool GetEncoderError(uint32_t& encoder_error) = 0;
  virtual bool GetControllerError(uint32_t& controller_error) = 0;
  virtual bool GetSensorlessError(uint32_t& sensorless_error) = 0;
  virtual bool GetEncoderEstimates(float& pos, float& vel) = 0;
  virtual bool GetEncoderCount(int32_t& shadow_cnt, int32_t& cnt_cpr) = 0;
  virtual bool GetIq(float& iq_setpoint, float& iq_measured) = 0;
  virtual bool GetSensorlessEstimates(float& pos, float& vel) = 0;
  virtual bool GetBusVoltageCurrent(float& bus_voltage, float& bus_current) = 0;
  // msg.rtr bit must NOT be set
  virtual bool GetAdcVoltage(float& adc_voltage) = 0;

  // Set functions
  virtual bool SetAxisNodeid(uint32_t axis_id) = 0;
  virtual bool SetAxisRequestedState(uint32_t state) = 0;
  virtual bool SetAxisStartupConfig() = 0;
  virtual bool SetInputPos(float input_pos, int16_t vel_ff,
			   int16_t torque_ff) = 0;
  virtual bool SetInputVel(float input_vel, float torque_ff) = 0;
  virtual bool SetInputTorque(float input_torque) = 0;
  virtual bool SetControllerModes(int32_t control_mode, int32_t input_mode) = 0;
  virtual bool SetLimits(float velocity_limit, float current_limit) = 0;
  virtual bool SetTrajVelLimit(float traj_vel_limit) = 0;
  virtual bool SetTrajAccelLimits(float traj_accel_limit,
				  float traj_decel_limit) = 0;
  virtual bool SetTrajInertia(float traj_inertia) = 0;
  virtual bool SetLinearCount(int32_t position) = 0;
  virtual bool SetPosGain(float pos_gain) = 0;
  virtual bool SetVelGains(float vel_gain, float vel_interator_gain) = 0;

  // Other functions
  virtual bool Nmt() = 0;
  virtual bool Estop() = 0;
  virtual bool ClearErrors() = 0;
  virtual bool StartAnticogging() = 0;
};

}  // namespace odrive
}  // namespace huron
