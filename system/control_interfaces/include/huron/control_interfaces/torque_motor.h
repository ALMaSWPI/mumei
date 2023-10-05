#pragma once

#include "huron/control_interfaces/motor.h"

namespace huron {

class TorqueMotor : public Motor {
 public:
  class TorqueMotorConfiguration : public MotorConfiguration {
   public:
    /**
     * Supports further inheritance.
     */
    TorqueMotorConfiguration(ConfigMap config_map,
                               std::set<std::string> valid_keys)
      : MotorConfiguration(config_map,
                           [&valid_keys]() {
                             std::set<std::string> tmp(kTorqueMotorValidKeys);
                             tmp.merge(valid_keys);
                             return tmp;
                           }()) {}

    TorqueMotorConfiguration()
      : TorqueMotorConfiguration({}, {}) {}

   private:
    static const inline std::set<std::string> kTorqueMotorValidKeys{};
  };

  explicit TorqueMotor(std::unique_ptr<TorqueMotorConfiguration> config)
      : Motor(std::move(config)) {}
  TorqueMotor() : TorqueMotor(std::make_unique<TorqueMotorConfiguration>()) {}
  TorqueMotor(const TorqueMotor&) = delete;
  TorqueMotor& operator=(const TorqueMotor&) = delete;
  virtual ~TorqueMotor() = default;
};

}  // namespace huron
