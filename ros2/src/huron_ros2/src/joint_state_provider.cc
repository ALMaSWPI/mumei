#include "huron_ros2/joint_state_provider.h"
#include "huron_ros2/huron_node.h"

namespace huron {
namespace ros2 {

JointStateProvider::JointStateProvider(
  size_t id_q, size_t nq, size_t id_v, size_t nv)
  : StateProvider(nq + nv, 1),
    nq_(nq),
    nv_(nv),
    id_q_(id_q),
    id_v_(id_v) {}

void JointStateProvider::RequestStateUpdate() {
  // Nothing to do
}

void JointStateProvider::GetNewState(
  Eigen::Ref<Eigen::MatrixXd> new_state) const {
  new_state = node_.lock()->GetJointState(id_q_, nq_, id_v_, nv_);
}

}  // namespace ros2
}  // namespace huron
