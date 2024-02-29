#include "huron/sensors/force_sensing_resistor.h"

namespace huron {

template <typename T>
ForceSensingResistor<T>::ForceSensingResistor(
  std::weak_ptr<const multibody::Frame<T>> frame)
  : SensorWithFrame<T>(1, 1, std::move(frame)) {}

template <typename T>
ForceSensingResistor<T>::ForceSensingResistor(
  std::weak_ptr<const multibody::Frame<T>> frame,
  std::unique_ptr<Configuration> config)
  : SensorWithFrame<T>(1, 1, std::move(frame), std::move(config)) {}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ForceSensingResistor)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ForceSensingResistor)
