#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/JointState.h>

#include <huron/control_interfaces/moving_group_component.h>


namespace huron {
namespace ros2 {

class Robot : public huron::MovingGroupComponent {
 public:
  class HuronNode : public rclcpp::Node {
   public:// init
    HuronNode() : Node("node_handler") {
      //TODO: Correct the topics and their msg type here
      //All the publisher(s)
      publisher_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
      //All the subscriber(s)
      subscription_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "sub_topic", 10, std::bind(&HuronNode::topic_callback_subscription_joints, this, _1));
    };

   private:
    void topic_callback_subscription_joints(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s' from encoder", msg->data.c_str());
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joints_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joints_;


  };

  void Initialize();
  void Terminate();

  bool Move(std::vector<float> values);
  bool MoveVelocity(std::vector<float> values);
  bool MoveTorque(std::vector<float> values);

  bool Stop();

 private:
};


}  // namespace huron
}