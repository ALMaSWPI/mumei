# include "huron_mujoco/huron.h"

namespace huron {
namespace mujoco {

Huron::Huron(std::unique_ptr<huron::RobotConfiguration> config)
  : huron::LeggedRobot(std::move(config)) {}

Huron::Huron()
  : Huron(std::make_unique<huron::RobotConfiguration>()) {}

void Huron::Initialize() {
}

void Huron::SetUp() {
}

void Huron::Terminate() {
}

}  // namespace ros2
}  // namespace huron
