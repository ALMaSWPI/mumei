#include "mumei/control_interfaces/sensor_with_frame.h"

namespace mumei {

SensorWithFrame::SensorWithFrame(const std::string& name,
                                 const Eigen::Vector2i& dim,
                                 std::weak_ptr<const multibody::Frame> frame)
  : Sensor(name, dim),
    frame_(std::move(frame)) {}

SensorWithFrame::SensorWithFrame(const std::string& name,
                                 const Eigen::Vector2i& dim,
                                 std::weak_ptr<const multibody::Frame> frame,
                                 std::unique_ptr<Configuration> config)
  : Sensor(name, dim, std::move(config)), frame_(std::move(frame)) {}

SensorWithFrame::SensorWithFrame(const std::string& name,
                                 int rows, int cols,
                                 std::weak_ptr<const multibody::Frame> frame)
  : Sensor(name, rows, cols),
    frame_(std::move(frame)) {}

SensorWithFrame::SensorWithFrame(const std::string& name,
                                 int rows, int cols,
                                 std::weak_ptr<const multibody::Frame> frame,
                                 std::unique_ptr<Configuration> config)
  : Sensor(name, rows, cols, std::move(config)), frame_(std::move(frame)) {}

}  // namespace mumei
