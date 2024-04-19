#include "huron_mujoco/floating_base.h"
#include "huron_mujoco/mujoco_env.h"

namespace huron {
namespace mujoco {

FloatingBase::FloatingBase(const std::string& name, int id,
                           std::weak_ptr<MujocoEnvironment> env)
    : StateProvider(13, 1), name_(name), id_(id), env_(env) {}

void FloatingBase::RequestStateUpdate() {
  states_ = env_.lock()->GetFloatingBaseStates(id_);
}

void FloatingBase::GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const {
  new_state = states_;
}

}  // namespace mujoco
}  // namespace huron
