#pragma once

#include <eigen3/Eigen/Core>
#include <memory>
#include <string>
#include "mumei/control_interfaces/sensor_with_frame.h"

namespace mumei {

class ForceSensingResistor : public SensorWithFrame {
 public:
  ForceSensingResistor(const std::string& name,
                                std::weak_ptr<const multibody::Frame> frame);
  ForceSensingResistor(const std::string& name,
                       std::weak_ptr<const multibody::Frame> frame,
                       std::unique_ptr<Configuration> config);
  ForceSensingResistor(const ForceSensingResistor&) = delete;
  ForceSensingResistor& operator=(const ForceSensingResistor&) = delete;
  ~ForceSensingResistor() override = default;
};

}  // namespace mumei
