#include "huron/sensors/force_torque_sensor.h"

namespace huron {

template <typename T>
ForceTorqueSensor<T>::ForceTorqueSensor(
  bool reverse_wrench_direction,
  std::weak_ptr<const multibody::Frame<T>> frame)
  : SensorWithFrame<T>(6, 1, std::move(frame)),
    reverse_wrench_direction_(reverse_wrench_direction) {}

template <typename T>
ForceTorqueSensor<T>::ForceTorqueSensor(
  bool reverse_wrench_direction,
  std::weak_ptr<const multibody::Frame<T>> frame,
  std::unique_ptr<Configuration> config)
  : SensorWithFrame<T>(6, 1, std::move(frame), std::move(config)),
    reverse_wrench_direction_(reverse_wrench_direction) {}

template <typename T>
void ForceTorqueSensor<T>::RequestStateUpdate() {
  wrench_ = DoGetWrenchRaw();
}

template <typename T>
void ForceTorqueSensor<T>::GetNewState(
  Eigen::Ref<huron::MatrixX<T>> new_state) const {
  new_state = GetValue();
}

template <typename T>
huron::VectorX<T> ForceTorqueSensor<T>::GetValue() const {
  if (reverse_wrench_direction_)
    return -wrench_;
  return wrench_;
}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ForceTorqueSensor)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ForceTorqueSensor)
