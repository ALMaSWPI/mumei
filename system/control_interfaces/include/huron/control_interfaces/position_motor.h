#pragma once

#include <set>
#include <string>
#include <utility>
#include <memory>

#include "huron/control_interfaces/motor.h"

namespace huron {

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

class PositionMotor : public Motor {
 public:
  PositionMotor(const std::string& name,
                std::unique_ptr<PositionMotorConfiguration> config,
                double gear_ratio)
    : Motor(name, std::move(config), gear_ratio) {}
  PositionMotor(const std::string& name, double gear_ratio)
    : Motor(name, gear_ratio) {}
  explicit PositionMotor(const std::string& name) : Motor(name) {}
  PositionMotor(const PositionMotor&) = delete;
  PositionMotor& operator=(const PositionMotor&) = delete;
  virtual ~PositionMotor() = default;
};

}  // namespace huron
