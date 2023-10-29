#pragma once

#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "huron/control_interfaces/generic_component.h"
#include "huron/control_interfaces/moving_group_component.h"

namespace huron {
namespace ros2 {

class HuronNode : public rclcpp::Node {
 public:
  HuronNode() : Node("node_handler") {
    subscription_joints_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        std::bind(
          &HuronNode::topic_callback_subscription_joints,
          this,
          std::placeholders::_1));
  }

  void topic_callback_subscription_joints(
    std::shared_ptr<const sensor_msgs::msg::JointState> msg) const {
    RCLCPP_INFO(this->get_logger(), "Joint positions:\n");
    for (auto& p : msg->position) {
      RCLCPP_INFO_STREAM(this->get_logger(), p << " ");
    }
    RCLCPP_INFO(this->get_logger(), "\n");
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    subscription_joints_;
};

class Robot : public huron::MovingGroupComponent {
 public:
  void Configure() override;
  void Initialize() override;
  void Terminate() override;
  void SetUp() override;

  bool Move(std::vector<float> values) override;
  bool Stop() override;

 private:
  HuronNode node;
};

}  // namespace ros2
}  // namespace huron
