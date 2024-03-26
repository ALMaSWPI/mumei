# include "huron_ros2/huron.h"

namespace huron {
namespace ros2 {

Huron::Huron(std::shared_ptr<HuronNode> node,
             std::unique_ptr<huron::RobotConfiguration> config)
  : huron::LeggedRobot(std::move(config)),
    node_(std::move(node)) {}

Huron::Huron(std::shared_ptr<HuronNode> node)
  : Huron(std::move(node),
          std::make_unique<huron::RobotConfiguration>()) {}

void Huron::Initialize() {
}

void Huron::SetUp() {
}

void Huron::Terminate() {
}

void Huron::Loop() {
  rclcpp::spin_some(node_);
}

}  // namespace ros2
}  // namespace huron
