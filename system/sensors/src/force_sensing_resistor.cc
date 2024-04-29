#include "huron/sensors/force_sensing_resistor.h"

namespace huron {

ForceSensingResistor::ForceSensingResistor(
  const std::string& name,
  std::weak_ptr<const multibody::Frame> frame)
  : SensorWithFrame(name, 1, 1, std::move(frame)) {}

ForceSensingResistor::ForceSensingResistor(
  const std::string& name,
  std::weak_ptr<const multibody::Frame> frame,
  std::unique_ptr<Configuration> config)
  : SensorWithFrame(name, 1, 1, std::move(frame), std::move(config)) {}

}  // namespace huron
