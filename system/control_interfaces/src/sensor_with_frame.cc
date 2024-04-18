#include "huron/control_interfaces/sensor_with_frame.h"

namespace huron {

template <typename T>
SensorWithFrame<T>::SensorWithFrame(const Eigen::Vector2i& dim,
                                 std::weak_ptr<const Frame> frame)
  : Sensor<T>(dim),
    frame_(std::move(frame)) {}

template <typename T>
SensorWithFrame<T>::SensorWithFrame(const Eigen::Vector2i& dim,
                                 std::weak_ptr<const Frame> frame,
                                 std::unique_ptr<Configuration> config)
  : Sensor<T>(dim, std::move(config)), frame_(std::move(frame)) {}

template <typename T>
SensorWithFrame<T>::SensorWithFrame(int rows, int cols,
                                 std::weak_ptr<const Frame> frame)
  : Sensor<T>(rows, cols),
    frame_(std::move(frame)) {}

template <typename T>
SensorWithFrame<T>::SensorWithFrame(int rows, int cols,
                                 std::weak_ptr<const Frame> frame,
                                 std::unique_ptr<Configuration> config)
  : Sensor<T>(rows, cols, std::move(config)), frame_(std::move(frame)) {}

}  // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::SensorWithFrame)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::SensorWithFrame)
