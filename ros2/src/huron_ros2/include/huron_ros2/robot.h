#pragma once

#include <string>

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "huron/control_interfaces/moving_group_component.h"


namespace huron {
namespace ros2 {
class HuronNode : public rclcpp::Node {
 public:// init
  HuronNode() : Node("node_handler") {
    //TODO: Correct the topics and their msg type here
    //All the publisher(s)
    publisher_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    //All the subscriber(s)
    subscription_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&HuronNode::topic_callback_subscription_joints, this, std::placeholders::_1));
  };

  void topic_callback_subscription_joints(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard something from joint states %s", msg->name);
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joints_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joints_;

};

class Robot : public huron::MovingGroupComponent {
 public:
  Robot(){};
  HuronNode node;
  void Configure();
  void Terminate();
  void SetUp(){};
  bool Move(std::vector<float> values);
  bool MoveVelocity(std::vector<float> values);
  bool MoveTorque(std::vector<float> values);

  bool Stop();

 private:
};



}  // namespace huron
}
