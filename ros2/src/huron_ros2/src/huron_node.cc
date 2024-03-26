#include "huron_ros2/huron_node.h"
#include "huron_ros2/force_torque_sensor.h"
#include "huron_ros2/joint_group_controller.h"
#include "huron_ros2/joint_state_provider.h"

namespace huron {
namespace ros2 {

HuronNode::HuronNode()
  : Node("huron_node")
{}

void HuronNode::AddJointStateProvider(std::shared_ptr<JointStateProvider> jsp,
                                      const std::string& topic,
                                      size_t nq,
                                      size_t nv,
                                      bool is_odom) {
  if (!is_odom) {
    // TODO(dtbpkmte): Create joint_group_state_provider and
    // single_joint_state_provider. This is just a workaround.
    if (joint_state_sub_ == nullptr) {
      joint_state_sub_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
          topic,
          10,
          std::bind(
            &HuronNode::JointStateCallback,
            this,
            std::placeholders::_1));
    }
  } else {
    assert(odom_sub_ == nullptr);
    odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
        topic,
        10,
        std::bind(&HuronNode::OdomCallback,
                  this,
                  std::placeholders::_1));
  }
  nq_ += nq;
  nv_ += nv;
  jsp->SetNode(std::static_pointer_cast<HuronNode>(shared_from_this()));
}

void HuronNode::AddForceTorqueSensor(
  std::shared_ptr<ForceTorqueSensor> ft_sensor,
  const std::string& topic) {
  size_t idx = wrench_stamped_subs_.size();
  ft_sensor->SetNode(std::static_pointer_cast<HuronNode>(shared_from_this()));
  ft_sensor->SetIndex(idx);
  wrench_stamped_subs_.push_back(
    this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topic,
      10,
      [this, idx](
          std::shared_ptr<const geometry_msgs::msg::WrenchStamped> msg) {
          this->WrenchStampedCallback(idx, msg);
      }));
}

void
HuronNode::AddJointGroupController(std::shared_ptr<JointGroupController> jgc,
                                   const std::string& topic) {
  size_t idx = float64_multi_array_pubs_.size();
  jgc->SetNode(std::static_pointer_cast<HuronNode>(shared_from_this()));
  jgc->SetPubIdx(idx);
  float64_multi_array_pubs_.push_back(
    this->create_publisher<std_msgs::msg::Float64MultiArray>(
      topic, 10));
}

void HuronNode::JointStateCallback(
  std::shared_ptr<const sensor_msgs::msg::JointState> msg) {
  for (size_t i = 0; i < msg->position.size(); ++i) {
    // 7 first elements are the floating base positions
    joint_state_(i + 7) = msg->position[i];
  }
  for (size_t j = 0; j < msg->velocity.size(); ++j) {
    // 6 first elements are the floating base velocities
    joint_state_(j + 6 + nq_) = msg->velocity[j];
  }
}

void HuronNode::OdomCallback(
  std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
  joint_state_.segment(0, 7) << msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z,
                                msg->pose.pose.orientation.w;
  joint_state_.segment(nq_, 6) << msg->twist.twist.linear.x,
                                  msg->twist.twist.linear.y,
                                  msg->twist.twist.linear.z,
                                  msg->twist.twist.angular.x,
                                  msg->twist.twist.angular.y,
                                  msg->twist.twist.angular.z;
}

void HuronNode::WrenchStampedCallback(
  size_t idx,
  std::shared_ptr<const geometry_msgs::msg::WrenchStamped> msg) {
  wrenches_[idx] << msg->wrench.force.x,
                    msg->wrench.force.y,
                    msg->wrench.force.z,
                    msg->wrench.torque.x,
                    msg->wrench.torque.y,
                    msg->wrench.torque.z;
}

void HuronNode::PublishFloat64MultiArray(size_t idx,
                                         const std::vector<double>& values) {
  std_msgs::msg::Float64MultiArray msg;
  msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  msg.layout.dim[0].size = values.size();
  msg.layout.dim[0].stride = 1;

  msg.data.insert(msg.data.end(), values.begin(), values.end());
  float64_multi_array_pubs_[idx]->publish(msg);
}

void HuronNode::Finalize() {
  // joint_state_ = Eigen::VectorXd::Zero(nq_ + nv_);
  joint_state_.resize(nq_ + nv_);
  wrenches_ = std::vector<huron::Vector6d>(wrench_stamped_subs_.size(),
                                           huron::Vector6d::Zero());
  finalized_ = true;
}

}  // namespace ros2
}  // namespace huron
