#pragma once

#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "huron/control_interfaces/robot.h"

namespace huron {
namespace ros2 {

class HuronNode : public rclcpp::Node {
 public:
  friend class Huron;

  HuronNode() : Node("huron_node") {
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

  void JointStatesCallback(
    std::shared_ptr<const sensor_msgs::msg::JointState> msg) const {
    // RCLCPP_INFO(this->get_logger(), "Joint positions:\n");
    // for (auto& p : msg->position) {
    //   RCLCPP_INFO_STREAM(this->get_logger(), p << " ");
    // }
    // RCLCPP_INFO(this->get_logger(), "\n");
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_effort_pub_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
};

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

  std::vector<double> GetPosition();
  std::vector<double> GetVelocity();

 private:
  std::shared_ptr<HuronNode> node_;
};

}  // namespace ros2
}  // namespace huron
