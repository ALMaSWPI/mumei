#pragma once

#include <string>
#include <memory>

#include "mumei/environment.h"
#include "mumei/control_interfaces/state_provider.h"
#include "mumei/control_interfaces/moving_interface.h"
#include "mumei/sensors/force_torque_sensor.h"
#include "mumei_node.h"

namespace mumei {
namespace ros2 {

class Ros2Environment : public Environment {
 public:
  Ros2Environment(std::function<void(void)> loop_func,
                  std::function<void(void)> exit_func = []() {});
  Ros2Environment(const Ros2Environment&) = delete;
  Ros2Environment& operator=(const Ros2Environment&) = delete;
  ~Ros2Environment() = default;

  void Initialize(int argc, char* argv[]) override;
  void Configure(void* config) override;
  void Finalize() override;
  void Exit() override;

  std::shared_ptr<mumei::StateProvider> CreateJointStateProvider(
    const std::string& name,
    const std::string& topic,
    size_t id_q, size_t nq,
    size_t id_v, size_t nv,
    bool is_odom = false);

  std::shared_ptr<mumei::ForceTorqueSensor> CreateForceTorqueSensor(
    const std::string& name,
    const std::string& topic,
    bool reverse_wrench_direction,
    std::weak_ptr<const multibody::Frame> frame);

  std::shared_ptr<mumei::MovingInterface> CreateJointGroupController(
    const std::string& topic,
    size_t dim);

 protected:
  void LoopPrologue() override;
  void LoopEpilogue() override;

 private:
  std::shared_ptr<MumeiNode> huron_node_;
};


}  // namespace ros2
}  // namespace mumei
