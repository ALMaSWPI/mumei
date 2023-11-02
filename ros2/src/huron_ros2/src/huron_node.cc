#include "huron_ros2/huron_node.h"

namespace huron {
namespace ros2 {

HuronNode::HuronNode()
  : Node("huron_node") {
  joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(
        &HuronNode::JointStatesCallback,
        this,
        std::placeholders::_1));
  joint_effort_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "joint_group_effort_controller/commands", 10);
}

void HuronNode::JointStatesCallback(
  std::shared_ptr<const sensor_msgs::msg::JointState> msg) {
  joint_position_ = msg->position;
  joint_velocity_ = msg->velocity;
}

void HuronNode::PublishJointEffort(const std::vector<double>& values) {
  std_msgs::msg::Float64MultiArray msg;
  msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  msg.layout.dim[0].size = values.size();
  msg.layout.dim[0].stride = 1;

  msg.data.insert(msg.data.end(), values.begin(), values.end());
  joint_effort_pub_->publish(msg);
}

}  // namespace ros2
}  // namespace huron
