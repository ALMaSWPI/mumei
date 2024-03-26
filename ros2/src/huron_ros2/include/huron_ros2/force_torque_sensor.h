#pragma once

#include <memory>
#include <utility>

#include "huron/sensors/force_torque_sensor.h"

namespace huron {
namespace ros2 {

class HuronNode;

class ForceTorqueSensor : public huron::ForceTorqueSensor {
  friend class HuronNode;
 public:
  ForceTorqueSensor(bool reverse_wrench_direction,
                    std::weak_ptr<const multibody::Frame> frame);
  ForceTorqueSensor(const ForceTorqueSensor&) = delete;
  ForceTorqueSensor& operator=(const ForceTorqueSensor&) = delete;
  ~ForceTorqueSensor() override = default;

  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

 protected:
  Vector6d DoGetWrenchRaw() override;

 private:
  size_t index_;
  std::weak_ptr<const HuronNode> node_;

  void SetNode(std::weak_ptr<HuronNode> node) {
    node_ = std::move(node);
  }

  void SetIndex(size_t index) {
    index_ = index;
  }
};

}  // namespace ros2
}  // namespace huron
