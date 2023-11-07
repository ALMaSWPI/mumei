#pragma once

#include "huron_node.h"

#include <string>
#include <vector>
#include <memory>

#include "huron/control_interfaces/robot.h"

namespace huron {
namespace ros2 {

class Huron : public huron::Robot {
 public:
  Huron(std::shared_ptr<HuronNode> node,
        std::unique_ptr<huron::Robot::RobotConfiguration> config);
  explicit Huron(std::shared_ptr<HuronNode> node);

  Huron(const Huron&) = delete;
  Huron& operator=(const Huron&) = delete;
  ~Huron() override = default;

  // GenericComponent interface
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  // MovingGroupComponent interface
  bool Move(const std::vector<double>& values) override;
  bool Stop() override;

  // Robot interface
  std::vector<double> GetJointPosition() override;
  std::vector<double> GetJointVelocity() override;

  //FSR
  std::vector<float> GetForceResistorSensorLeft();
  std::vector<float> GetForceResistorSensorRight();

  // ROS-specific
  void Loop();

 private:
  std::shared_ptr<HuronNode> node_;
};

}  // namespace ros2
}  // namespace huron
