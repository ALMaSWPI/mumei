#pragma once

#include <memory>
#include <utility>
#include <string>

#include "mumei/control_interfaces/state_provider.h"

namespace mumei {
namespace ros2 {

class MumeiNode;

class JointStateProvider : public mumei::StateProvider {
  friend class MumeiNode;
 public:
  JointStateProvider(const std::string& name,
                     size_t id_q, size_t nq,
                     size_t id_v, size_t nv);

  void RequestStateUpdate() override;
  void GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const override;

 private:
  std::weak_ptr<MumeiNode> node_;
  size_t nq_;
  size_t nv_;
  size_t id_q_;
  size_t id_v_;

  void SetNode(std::weak_ptr<MumeiNode> node) {
    node_ = std::move(node);
  }
};

}  // namespace ros2
}  // namespace mumei
