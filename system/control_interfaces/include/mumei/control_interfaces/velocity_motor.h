#pragma once

#include <set>
#include <string>
#include <utility>
#include <memory>

#include "mumei/control_interfaces/motor.h"

namespace mumei {

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

class VelocityMotor : public Motor {
 public:
  VelocityMotor(const std::string& name,
                std::unique_ptr<VelocityMotorConfiguration> config,
                double gear_ratio)
    : Motor(name, std::move(config), gear_ratio) {}
  VelocityMotor(const std::string& name, double gear_ratio)
    : Motor(name, gear_ratio) {}
  explicit VelocityMotor(const std::string& name) : Motor(name) {}
  VelocityMotor(const VelocityMotor&) = delete;
  VelocityMotor& operator=(const VelocityMotor&) = delete;
  ~VelocityMotor() override = default;
};

}  // namespace mumei
