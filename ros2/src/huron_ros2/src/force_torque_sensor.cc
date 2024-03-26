#include "huron_ros2/force_torque_sensor.h"
#include "huron_ros2/huron_node.h"

namespace huron {
namespace ros2 {

ForceTorqueSensor::ForceTorqueSensor(
  bool reverse_wrench_direction,
  std::weak_ptr<const multibody::Frame> frame)
  : huron::ForceTorqueSensor(reverse_wrench_direction, std::move(frame)) {}

void ForceTorqueSensor::Initialize() {
}

void ForceTorqueSensor::SetUp() {
}

void ForceTorqueSensor::Terminate() {
}

Vector6d ForceTorqueSensor::DoGetWrenchRaw() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("HuronNode is expired.");
  }
  return node->GetWrench(index_);
}

}  // namespace ros2
}  // namespace huron
