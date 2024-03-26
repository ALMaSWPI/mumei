#pragma once

#include <memory>
#include <utility>

#include "huron/control_interfaces/state_provider.h"

namespace huron {
namespace ros2 {

class HuronNode;

class JointStateProvider : public huron::StateProvider {
  friend class HuronNode;
 public:
  JointStateProvider(size_t id_q, size_t nq, size_t id_v, size_t nv);

  void RequestStateUpdate() override;
  void GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const override;

 private:
  std::weak_ptr<HuronNode> node_;
  size_t nq_;
  size_t nv_;
  size_t id_q_;
  size_t id_v_;

  void SetNode(std::weak_ptr<HuronNode> node) {
    node_ = std::move(node);
  }
};

}  // namespace ros2
}  // namespace huron
