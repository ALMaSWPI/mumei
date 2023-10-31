# include "huron_ros2/huron.h"

namespace huron {
namespace ros2 {

Huron::Huron(std::shared_ptr<HuronNode> node,
      std::unique_ptr<huron::Robot::RobotConfiguration> config)
  : huron::Robot(std::move(config)), node_(std::move(node)) {}

Huron::Huron(std::shared_ptr<HuronNode> node)
  : Huron(std::move(node),
          std::make_unique<huron::Robot::RobotConfiguration>()) {}

void Huron::Initialize() {
}

void Huron::SetUp() {
}

void Huron::Terminate() {
}

bool Huron::Move(const std::vector<double>& values) {
  sensor_msgs::msg::JointState msg;
  msg.set__effort(values);
  return true;
}

bool Huron::Stop() {
  return Move(std::vector<double>(12));
}

std::vector<double> Huron::GetPosition() {
  return node_->joint_position_;
}

std::vector<double> Huron::GetVelocity() {
  return node_->joint_velocity_;
}

}  // namespace ros2
}  // namespace huron
