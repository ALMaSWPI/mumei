#pragma once

#include <vector>
#include <set>
#include <string>
#include <utility>
#include <memory>

#include "configuration.h"
#include "moving_group_component.h"
#include "joint.h"

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

  explicit Robot(std::unique_ptr<RobotConfiguration> config);
  Robot();
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  ~Robot() override = default;

  // Robot methods
  virtual std::vector<double> GetJointPosition();
  virtual std::vector<double> GetJointVelocity();

  virtual bool AddJoint(std::shared_ptr<Joint> joint);

  // GenericComponent methods
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  // MovingGroupComponent methods
  bool Move(const std::vector<double>& values) override;
  bool Stop() override;

 private:
  std::vector<std::shared_ptr<Joint>> joints_;
};

}  // namespace huron
