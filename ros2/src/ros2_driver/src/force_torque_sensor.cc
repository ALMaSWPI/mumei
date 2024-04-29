#include "ros2_driver/force_torque_sensor.h"
#include "ros2_driver/mumei_node.h"

namespace mumei {
namespace ros2 {

ForceTorqueSensor::ForceTorqueSensor(
  const std::string& name,
  bool reverse_wrench_direction,
  std::weak_ptr<const multibody::Frame> frame)
  : mumei::ForceTorqueSensor(name,
                             reverse_wrench_direction,
                             std::move(frame)) {}

void ForceTorqueSensor::Initialize() {
}

void ForceTorqueSensor::SetUp() {
}

void ForceTorqueSensor::Terminate() {
}

Vector6d ForceTorqueSensor::DoGetWrenchRaw() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("MumeiNode is expired.");
  }
  return node->GetWrench(index_);
}

}  // namespace ros2
}  // namespace mumei
