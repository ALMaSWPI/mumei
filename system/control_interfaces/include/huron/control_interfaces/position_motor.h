#pragma once

#include "huron/control_interfaces/motor.h"

namespace huron {

class PositionMotor : public Motor {
 public:
  class PositionMotorConfiguration : public MotorConfiguration {
   public:
    /**
     * Supports further inheritance.
     */
    PositionMotorConfiguration(ConfigMap config_map,
                               std::set<std::string> valid_keys)
      : MotorConfiguration(config_map,
                           [&valid_keys]() {
                             std::set<std::string> tmp(kPositionMotorValidKeys);
                             tmp.merge(valid_keys);
                             return tmp;
                           }()) {}

    PositionMotorConfiguration()
      : PositionMotorConfiguration({}, {}) {}

   private:
    static const inline std::set<std::string> kPositionMotorValidKeys{};
  };

  explicit PositionMotor(std::unique_ptr<PositionMotorConfiguration> config)
      : Motor(std::move(config)) {}
  PositionMotor() : PositionMotor(std::make_unique<PositionMotorConfiguration>()) {}
  PositionMotor(const PositionMotor&) = delete;
  PositionMotor& operator=(const PositionMotor&) = delete;
  virtual ~PositionMotor() = default;
};

}  // namespace huron
