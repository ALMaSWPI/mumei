#pragma once

#include <vector>
#include <memory>
#include <utility>

#include "huron/control_interfaces/moving_interface.h"

namespace huron {
namespace ros2 {

class HuronNode;

class JointGroupController : public huron::MovingInterface {
  friend class HuronNode;
 public:
  explicit JointGroupController(size_t dim);
  JointGroupController(const JointGroupController&) = delete;
  JointGroupController& operator=(const JointGroupController&) = delete;
  ~JointGroupController() override = default;

  bool Move(const std::vector<double>& values) override;
  bool Move(const Eigen::VectorXd& values) override;
  bool Stop() override;

 private:
  std::weak_ptr<HuronNode> node_;
  /// Dimension of the actuated joint group
  size_t dim_;
  /// Publisher index
  size_t pub_idx_;

  void SetNode(std::weak_ptr<HuronNode> node) {
    node_ = std::move(node);
  }

  void SetPubIdx(size_t pub_idx) {
    pub_idx_ = pub_idx;
  }
};

}  // namespace ros2
}  // namespace huron
