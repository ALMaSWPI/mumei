#include "ros2_driver/joint_state_provider.h"
#include "ros2_driver/mumei_node.h"

namespace mumei {
namespace ros2 {

JointStateProvider::JointStateProvider(
  const std::string& name,
  size_t id_q, size_t nq, size_t id_v, size_t nv)
  : StateProvider(name, nq + nv, 1),
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
}  // namespace mumei
