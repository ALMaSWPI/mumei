#include "huron_ros2/joint_group_controller.h"
#include "huron_ros2/huron_node.h"

namespace huron {
namespace ros2 {

JointGroupController::JointGroupController(size_t dim)
  : huron::MovingInterface(dim), dim_(dim) {}

bool JointGroupController::Move(const std::vector<double>& values) {
  assert(values.size() == dim_);
  node_.lock()->PublishFloat64MultiArray(pub_idx_, values);
  return true;
}

bool JointGroupController::Move(const Eigen::VectorXd& values) {
  assert(static_cast<size_t>(values.size()) == dim_);
  node_.lock()->PublishFloat64MultiArray(
    pub_idx_,
    std::vector<double>(values.data(), values.data() + values.size()));
  return true;
}

bool JointGroupController::Stop() {
  return Move(std::vector<double>(dim_));
}


}  // namespace ros2
}  // namespace huron
