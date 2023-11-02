#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace huron {
namespace ros2 {

class HuronNode : public rclcpp::Node {
 public:
  friend class Huron;

  HuronNode();

  void JointStatesCallback(
    std::shared_ptr<const sensor_msgs::msg::JointState> msg);
  void PublishJointEffort(const std::vector<double>& values);

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_effort_pub_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
};

}  // namespace ros2
}  // namespace huron
