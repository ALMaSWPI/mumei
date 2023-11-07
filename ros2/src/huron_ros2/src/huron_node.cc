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
  fsr_left_sub_ =
    this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "huron/sensor/l1_ft_sensor",
      10,
      std::bind(
        &HuronNode::FSRLeftCallback,
        this,
        std::placeholders::_1));
  fsr_right_sub_ =
    this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "huron/sensor/r1_ft_sensor",
      10,
      std::bind(
        &HuronNode::FSRRightCallback,
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

void HuronNode::FSRLeftCallback(
  std::shared_ptr<const geometry_msgs::msg::WrenchStamped> msg) {
  geometry_msgs::msg::Wrench msgTemp = msg->wrench;
  fsr_left_.push_back(msgTemp.force.x);
  fsr_left_.push_back(msgTemp.force.y);
  fsr_left_.push_back(msgTemp.force.z);
  fsr_left_.push_back(msgTemp.torque.x);
  fsr_left_.push_back(msgTemp.torque.y);
  fsr_left_.push_back(msgTemp.torque.z);
}

void HuronNode::FSRRightCallback(
  std::shared_ptr<const geometry_msgs::msg::WrenchStamped> msg) {
  geometry_msgs::msg::Wrench msgTemp = msg->wrench;
  fsr_right_.push_back(msgTemp.force.x);
  fsr_right_.push_back(msgTemp.force.y);
  fsr_right_.push_back(msgTemp.force.z);
  fsr_right_.push_back(msgTemp.torque.x);
  fsr_right_.push_back(msgTemp.torque.y);
  fsr_right_.push_back(msgTemp.torque.z);
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
