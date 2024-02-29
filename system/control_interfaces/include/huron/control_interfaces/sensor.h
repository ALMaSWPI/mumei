#pragma once

#include <eigen3/Eigen/Core>

#include <memory>

#include "huron/control_interfaces/generic_component.h"
#include "huron/control_interfaces/state_provider.h"

namespace huron {

template <typename T>
class Sensor : public GenericComponent, public StateProvider<T> {
 public:
  Sensor(const Eigen::Vector2i& dim,
         std::unique_ptr<Configuration> config);
  explicit Sensor(const Eigen::Vector2i& dim);
  Sensor(int rows, int cols,
         std::unique_ptr<Configuration> config);
  Sensor(int rows, int cols);
  Sensor(const Sensor&) = delete;
  Sensor& operator=(const Sensor&) = delete;
  virtual ~Sensor() = default;

  /**
   * @brief Get the sensor value.
   */
  virtual huron::VectorX<T> GetValue() const;
  virtual huron::VectorX<T> ReloadAndGetValue();
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::Sensor)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::Sensor)
