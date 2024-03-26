#pragma once

#include <eigen3/Eigen/Core>

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "huron/types.h"


namespace huron {
namespace ros2 {

class ForceTorqueSensor;
class JointGroupController;
class JointStateProvider;

class HuronNode : public rclcpp::Node {
 public:
  HuronNode();
  ~HuronNode() override = default;

  void JointStateCallback(
    std::shared_ptr<const sensor_msgs::msg::JointState> msg);
  void OdomCallback(
    std::shared_ptr<const nav_msgs::msg::Odometry> msg);
  void WrenchStampedCallback(
    size_t idx,
    std::shared_ptr<const geometry_msgs::msg::WrenchStamped> msg);
  void PublishFloat64MultiArray(size_t idx, const std::vector<double>& values);

  const huron::Vector6d& GetWrench(size_t idx) const {
    return wrenches_[idx];
  }

  /**
   * @brief Add a subscriber to the joint state topic. There can be at most one
   * subscriber to the joint state topic.
   */
  void AddJointStateProvider(
    std::shared_ptr<JointStateProvider> jsp,
    const std::string& topic,
    size_t nq, size_t nv,
    bool is_odom = false);
  void AddForceTorqueSensor(
    std::shared_ptr<ForceTorqueSensor> ft_sensor,
    const std::string& topic);
  void AddJointGroupController(
    std::shared_ptr<JointGroupController> jgc,
    const std::string& topic);

  /**
   * @brief Finalize the configuration. This method must be called after adding
   * all the ROS2 components to the node, else exceptions will be thrown.
   */
  void Finalize();

  Eigen::VectorXd GetJointState(size_t id_q, size_t dim_q,
                                size_t id_v, size_t dim_v) const {
    Eigen::VectorXd state(dim_q + dim_v);
    state.segment(0, dim_q) = joint_state_.segment(id_q, dim_q);
    state.segment(dim_q, dim_v) = joint_state_.segment(nq_ + id_v,
                                                       dim_v);
    return state;
  }

 private:
  bool finalized_ = false;
  /// Dimension of the joint position, including the floating base
  size_t nq_ = 0;
  /// Dimension of the joint velocity, including the floating base
  size_t nv_ = 0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr>
    float64_multi_array_pubs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>
    ::SharedPtr> wrench_stamped_subs_;

  Eigen::VectorXd joint_state_;
  std::vector<huron::Vector6d> wrenches_;
};

}  // namespace ros2
}  // namespace huron
