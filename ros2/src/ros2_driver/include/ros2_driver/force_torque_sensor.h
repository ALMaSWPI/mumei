#pragma once

#include <memory>
#include <utility>
#include <string>

#include "mumei/sensors/force_torque_sensor.h"

namespace mumei {
namespace ros2 {

class MumeiNode;

class ForceTorqueSensor : public mumei::ForceTorqueSensor {
  friend class MumeiNode;
 public:
  ForceTorqueSensor(const std::string& name,
                    bool reverse_wrench_direction,
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
  std::weak_ptr<const MumeiNode> node_;

  void SetNode(std::weak_ptr<MumeiNode> node) {
    node_ = std::move(node);
  }

  void SetIndex(size_t index) {
    index_ = index;
  }
};

}  // namespace ros2
}  // namespace mumei
