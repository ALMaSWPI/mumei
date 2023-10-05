#pragma once

#include "huron/control_interfaces/motor.h"

namespace huron {

class VelocityMotor : public Motor {
 public:
  class VelocityMotorConfiguration : public MotorConfiguration {
   public:
    /**
     * Supports further inheritance.
     */
    VelocityMotorConfiguration(ConfigMap config_map,
                               std::set<std::string> valid_keys)
      : MotorConfiguration(config_map,
                           [&valid_keys]() {
                             std::set<std::string> tmp(kVelocityMotorValidKeys);
                             tmp.merge(valid_keys);
                             return tmp;
                           }()) {}

    VelocityMotorConfiguration()
      : VelocityMotorConfiguration({}, {}) {}

   private:
    static const inline std::set<std::string> kVelocityMotorValidKeys{};
  };

  explicit VelocityMotor(std::unique_ptr<VelocityMotorConfiguration> config)
      : Motor(std::move(config)) {}
  VelocityMotor() : VelocityMotor(std::make_unique<VelocityMotorConfiguration>()) {}
  VelocityMotor(const VelocityMotor&) = delete;
  VelocityMotor& operator=(const VelocityMotor&) = delete;
  virtual ~VelocityMotor() = default;
};

}  // namespace huron
