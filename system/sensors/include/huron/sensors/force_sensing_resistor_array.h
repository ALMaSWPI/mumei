#pragma once

#include <eigen3/Eigen/Core>

#include <string>
#include <memory>
#include <vector>

#include "huron/control_interfaces/sensor_with_frame.h"
#include "huron/sensors/force_sensing_resistor.h"

namespace huron {

template <typename T>
class ForceSensingResistorArray : public SensorWithFrame<T> {
 public:
  ForceSensingResistorArray(
    const std::string& name,
    std::weak_ptr<const multibody::Frame<T>> frame,
    const std::vector<std::shared_ptr<ForceSensingResistor<T>>>& fsr_array);
  ForceSensingResistorArray(
    const std::string& name,
    std::weak_ptr<const multibody::Frame<T>> frame,
    const std::vector<std::shared_ptr<ForceSensingResistor<T>>>& fsr_array,
    std::unique_ptr<Configuration> config);

  ForceSensingResistorArray(const ForceSensingResistorArray&) = delete;
  ForceSensingResistorArray&
    operator=(const ForceSensingResistorArray&) = delete;
  ~ForceSensingResistorArray() override = default;

  void RequestStateUpdate() override;

  void GetNewState(Eigen::Ref<huron::MatrixX<T>> new_state) const override;

  huron::SE3<T> GetSensorPose(size_t index) const;

  size_t num_sensors() const { return fsr_array_.size(); }

 protected:
  std::string name_;
  huron::VectorX<T> values_;
  std::vector<std::shared_ptr<ForceSensingResistor<T>>> fsr_array_;
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::ForceSensingResistorArray)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::ForceSensingResistorArray)
