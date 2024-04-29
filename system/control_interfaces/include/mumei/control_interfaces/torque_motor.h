#pragma once

#include <set>
#include <string>
#include <utility>
#include <memory>

#include "mumei/control_interfaces/motor.h"

namespace mumei {

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

class TorqueMotor : public Motor {
 public:
  TorqueMotor(const std::string& name,
              std::unique_ptr<TorqueMotorConfiguration> config,
              double gear_ratio)
    : Motor(name, std::move(config), gear_ratio) {}
  TorqueMotor(const std::string& name,
              double gear_ratio)
    : Motor(name, gear_ratio) {}
  explicit TorqueMotor(const std::string& name) : Motor(name) {}
  TorqueMotor(const TorqueMotor&) = delete;
  TorqueMotor& operator=(const TorqueMotor&) = delete;
  ~TorqueMotor() override = default;
};

}  // namespace mumei
