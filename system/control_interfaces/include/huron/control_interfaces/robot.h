#pragma once

#include <vector>
#include <memory>

#include "configuration.h"
#include "moving_group_component.h"

namespace huron {

class Robot : public MovingGroupComponent {
 public:
  class RobotConfiguration : public Configuration {
   private:
    static const inline std::set<std::string> kRobotValidKeys{};

   public:
    RobotConfiguration(ConfigMap config_map,
                       std::set<std::string> valid_keys)
        : Configuration(config_map, [&valid_keys]() {
                          std::set<std::string> tmp(kRobotValidKeys);
                          tmp.merge(valid_keys);
                          return tmp;
                        }()) {}

    explicit RobotConfiguration(ConfigMap config_map)
        : RobotConfiguration(config_map, {}) {}

    RobotConfiguration()
        : RobotConfiguration({}, {}) {}
  };

  explicit Robot(std::unique_ptr<RobotConfiguration> config)
      : MovingGroupComponent(std::move(config)) {}
  Robot() : Robot(std::make_unique<RobotConfiguration>()) {}
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  ~Robot() override = default;
};

}  // namespace huron
