#pragma once

#include <vector>
#include <memory>
#include <utility>

#include "mumei/control_interfaces/moving_interface.h"

namespace mumei {
namespace ros2 {

class MumeiNode;

class JointGroupController : public mumei::MovingInterface {
  friend class MumeiNode;
 public:
  explicit JointGroupController(size_t dim);
  JointGroupController(const JointGroupController&) = delete;
  JointGroupController& operator=(const JointGroupController&) = delete;
  ~JointGroupController() override = default;

  bool Move(const std::vector<double>& values) override;
  bool Move(const Eigen::VectorXd& values) override;
  bool Stop() override;

 private:
  std::weak_ptr<MumeiNode> node_;
  /// Dimension of the actuated joint group
  size_t dim_;
  /// Publisher index
  size_t pub_idx_;

  void SetNode(std::weak_ptr<MumeiNode> node) {
    node_ = std::move(node);
  }

  void SetPubIdx(size_t pub_idx) {
    pub_idx_ = pub_idx;
  }
};

}  // namespace ros2
}  // namespace mumei
