#pragma once

#include <eigen3/Eigen/Core>
#include <memory>
#include "huron/control_interfaces/sensor_with_frame.h"

namespace huron {

template <typename T>
class ForceSensingResistor : public SensorWithFrame<T> {
 public:
  explicit ForceSensingResistor(std::weak_ptr<const multibody::Frame<T>> frame);
  ForceSensingResistor(std::weak_ptr<const multibody::Frame<T>> frame,
                       std::unique_ptr<Configuration> config);
  ForceSensingResistor(const ForceSensingResistor&) = delete;
  ForceSensingResistor& operator=(const ForceSensingResistor&) = delete;
  ~ForceSensingResistor() override = default;
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ForceSensingResistor)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ForceSensingResistor)
