#pragma once

#include <eigen3/Eigen/Core>

#include <memory>

#include "huron/utils/template_instantiations.h"
#include "huron/control_interfaces/sensor.h"
#include "huron/multibody/frame.h"

namespace huron {

template <typename T>
class SensorWithFrame : public Sensor<T> {
  using Frame = multibody::Frame<T>;

 public:
  SensorWithFrame(const Eigen::Vector2i& dim,
                  std::weak_ptr<const Frame> frame);
  SensorWithFrame(const Eigen::Vector2i& dim,
                  std::weak_ptr<const Frame> frame,
                  std::unique_ptr<Configuration> config);
  SensorWithFrame(int rows, int cols,
                  std::weak_ptr<const Frame> frame);
  SensorWithFrame(int rows, int cols,
                  std::weak_ptr<const Frame> frame,
                  std::unique_ptr<Configuration> config);
  SensorWithFrame(const SensorWithFrame&) = delete;
  SensorWithFrame& operator=(const SensorWithFrame&) = delete;
  ~SensorWithFrame() override = default;

  /**
   * @brief Get the sensor frame.
   */
  std::weak_ptr<const Frame> GetSensorFrame() const {
    return frame_;
  }

 private:
  std::weak_ptr<const Frame> frame_;
};

}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::SensorWithFrame)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::SensorWithFrame)
