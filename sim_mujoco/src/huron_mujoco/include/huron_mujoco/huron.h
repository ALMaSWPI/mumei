#pragma once
#include <string>
#include <vector>
#include <memory>

#include "huron/control_interfaces/legged_robot.h"
#include <mujoco/mujoco.h>


namespace huron {
namespace mujoco {

class Huron : public huron::LeggedRobot {
 public:
  Huron(std::unique_ptr<huron::RobotConfiguration> config);
  explicit Huron();

  Huron(const Huron&) = delete;
  Huron& operator=(const Huron&) = delete;
  ~Huron() override = default;


  // GenericComponent interface
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  // MovingGroupComponent interface
  bool Move(const std::vector<double>& values) override;
  bool Stop() override;

  // Mujoco-specific
  void BuildFromXml(const char* xml_path);

 private:
  mjModel* m_;                  // MuJoCo model
  mjData* d_;                   // MuJoCo data
};

}  // namespace ros2
}  // namespace huron
