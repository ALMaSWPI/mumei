#include "huron/control_interfaces/joint.h"

namespace huron {

template <typename T>
Joint<T>::Joint(std::unique_ptr<JointDescription> joint_desc,
             std::shared_ptr<StateProvider<T>> state_provider)
  : jd_(std::move(joint_desc)),
    positions_(huron::VectorX<T>::Zero(jd_->num_positions())),
    velocities_(huron::VectorX<T>::Zero(jd_->num_velocities())),
    state_provider_(std::move(state_provider)) {}

template <typename T>
void Joint<T>::SetStateProvider(std::shared_ptr<StateProvider<T>> state_provider) {
  assert(
    state_provider->dim()[0] == (jd_->num_positions() + jd_->num_velocities())
    &&
    state_provider->dim()[1] == 1);
  state_provider_ = state_provider;
}

template <typename T>
void Joint<T>::UpdateState() {
  assert(state_provider_ != nullptr);
  state_provider_->RequestStateUpdate();
  huron::VectorX<T> tmp(jd_->num_positions() + jd_->num_velocities());
  state_provider_->GetNewState(tmp);
  positions_ = tmp.segment(0, jd_->num_positions());
  velocities_ = tmp.segment(jd_->num_positions(), jd_->num_velocities());
}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::Joint)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::Joint)
