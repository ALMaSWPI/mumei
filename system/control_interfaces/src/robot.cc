#include "huron/control_interfaces/robot.h"

namespace huron {

template <typename T>
Robot<T>::Robot(std::unique_ptr<RobotConfiguration> config)
  : GenericComponent(std::move(config)), MovingGroup(),
    model_(std::make_shared<Model>()) {}

template <typename T>
Robot<T>::Robot()
  : Robot<T>::Robot(std::make_unique<RobotConfiguration>()) {}

template <typename T>
Robot<T>::Robot(std::unique_ptr<RobotConfiguration> config,
             std::shared_ptr<Model> model)
  : GenericComponent(std::move(config)), MovingGroup(),
    model_(std::make_shared<Model>()) {}

template <typename T>
Robot<T>::Robot(std::shared_ptr<Model> model)
  : Robot<T>::Robot(std::make_unique<RobotConfiguration>(), std::move(model)) {}

template <typename T>
void Robot<T>::RegisterStateProvider(
  std::shared_ptr<StateProvider<T>> state_provider,
  bool is_joint_state_provider) {
  if (!is_joint_state_provider) {
    non_joint_state_providers_.push_back(state_provider);
  } else {
    joint_state_providers_.push_back(state_provider);
  }
}

template <typename T>
void Robot<T>::UpdateAllStates() {
  for (auto& state_provider : non_joint_state_providers_) {
    state_provider->RequestStateUpdate();
  }
  model_->UpdateJointStates();
}

template <typename T>
void Robot<T>::UpdateJointStates() {
  model_->UpdateJointStates();
}

template <typename T>
const Eigen::VectorBlock<const huron::VectorX<T>>
Robot<T>::GetJointPositions() const {
  return model_->GetPositions();
}

template <typename T>
const Eigen::VectorBlock<const huron::VectorX<T>>
Robot<T>::GetJointVelocities() const {
  return model_->GetVelocities();
}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::Robot)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::Robot)
