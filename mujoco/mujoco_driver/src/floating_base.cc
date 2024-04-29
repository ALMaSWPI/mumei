#include "mumei/mujoco/floating_base.h"
#include "mumei/mujoco/mujoco_env.h"

namespace mumei {
namespace mujoco {

FloatingBase::FloatingBase(const std::string& name,
                           int mj_id,
                           std::weak_ptr<MujocoEnvironment> env)
    : StateProvider(name, 13, 1), mj_id_(mj_id), env_(env) {}

void FloatingBase::RequestStateUpdate() {
  states_ = env_.lock()->GetFloatingBaseStates(mj_id_);
}

void FloatingBase::GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const {
  new_state = states_;
}

}  // namespace mujoco
}  // namespace mumei
