#include "huron/control_interfaces/sensor.h"
#include <memory>

namespace huron {

template <typename T>
Sensor<T>::Sensor(const Eigen::Vector2i& dim,
               std::unique_ptr<Configuration> config)
    : GenericComponent(std::move(config)),
      StateProvider<T>(dim) {}

template <typename T>
Sensor<T>::Sensor(const Eigen::Vector2i& dim)
  : GenericComponent(),
    StateProvider<T>(dim) {}

template <typename T>
Sensor<T>::Sensor(int rows, int cols,
                  std::unique_ptr<Configuration> config)
    : GenericComponent(std::move(config)),
      StateProvider<T>(rows, cols) {}

template <typename T>
Sensor<T>::Sensor(int rows, int cols)
  : GenericComponent(),
    StateProvider<T>(rows, cols) {}

template <typename T>
huron::VectorX<T> Sensor<T>::GetValue() const {
  huron::VectorX<T> tmp;
  this->GetNewState(tmp);
  return tmp;
}

template <typename T>
huron::VectorX<T> Sensor<T>::ReloadAndGetValue() {
  this->RequestStateUpdate();
  return GetValue();
}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::Sensor)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::Sensor)
