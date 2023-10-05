#pragma once

#include <vector>

#include "moving_component.h"

namespace huron {

/**
 * Abstract class for a motor.
 *
 * A motor takes in input value(s) and actuates accordingly. It does not
 * need to know its position or velocity; that is the job of encoders
 * and other sensors.
 *
 * @ingroup control_interfaces
 */
class Motor : public MovingComponent {
 public:
  class MotorConfiguration : public Configuration {
   private:
    static const inline std::set<std::string> kMotorValidKeys{};

   public:
    MotorConfiguration(ConfigMap config_map,
                       std::set<std::string> valid_keys)
        : Configuration(config_map, [&valid_keys]() {
                          std::set<std::string> tmp(kMotorValidKeys);
                          tmp.merge(valid_keys);
                          return tmp;
                        }()) {}

    explicit MotorConfiguration(ConfigMap config_map)
        : MotorConfiguration(config_map, {}) {}

    MotorConfiguration()
        : MotorConfiguration({}, {}) {}
  };

  explicit Motor(std::unique_ptr<MotorConfiguration> config)
      : MovingComponent(std::move(config)) {}
  Motor() : Motor(std::make_unique<MotorConfiguration>()) {}
  Motor(const Motor&) = delete;
  Motor& operator=(const Motor&) = delete;
  ~Motor() override = default;
};

}  // namespace huron
