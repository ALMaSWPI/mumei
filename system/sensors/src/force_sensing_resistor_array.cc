#include "huron/sensors/force_sensing_resistor_array.h"

namespace huron {

template <typename T>
ForceSensingResistorArray<T>::ForceSensingResistorArray(
  const std::string& name,
  std::weak_ptr<const multibody::Frame<T>> frame,
  const std::vector<std::shared_ptr<ForceSensingResistor<T>>>& fsr_array)
  : SensorWithFrame<T>(fsr_array.size(), 1, std::move(frame)),
    name_(name),
    values_(huron::VectorX<T>::Zero(fsr_array.size())),
    fsr_array_(fsr_array) {}

template <typename T>
ForceSensingResistorArray<T>::ForceSensingResistorArray(
  const std::string& name,
  std::weak_ptr<const multibody::Frame<T>> frame,
  const std::vector<std::shared_ptr<ForceSensingResistor<T>>>& fsr_array,
  std::unique_ptr<Configuration> config)
  : SensorWithFrame<T>(fsr_array.size(), 1, std::move(frame), std::move(config)),
    name_(name),
    values_(huron::VectorX<T>::Zero(fsr_array.size())),
    fsr_array_(fsr_array) {}

template <typename T>
void ForceSensingResistorArray<T>::RequestStateUpdate() {
  for (size_t i = 0; i < fsr_array_.size(); ++i) {
    values_(i) = fsr_array_[i]->ReloadAndGetValue()(0);
  }
}

template <typename T>
void ForceSensingResistorArray<T>::GetNewState(
  Eigen::Ref<huron::MatrixX<T>> new_state) const {
  new_state = values_;
}

template <typename T>
huron::SE3<T> ForceSensingResistorArray<T>::GetSensorPose(size_t index) const {
  return fsr_array_[index]->GetSensorFrame().lock()->GetTransformInWorld();
}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ForceSensingResistorArray)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ForceSensingResistorArray)
