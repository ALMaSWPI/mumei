#include "huron_mujoco/motor.h"
#include "huron_mujoco/mujoco_env.h"

namespace huron {
namespace mujoco {

Motor::Motor(const std::string& name, int id,
             std::weak_ptr<MujocoEnvironment> env, double gear_ratio)
    : huron::Motor(gear_ratio), name_(name), id_(id), env_(env) {}

void Motor::Initialize() {
}

void Motor::SetUp() {
}

void Motor::Terminate() {
}

bool Motor::Move(const std::vector<double>& values) {
  return Move(values[0]);
}

bool Motor::Move(const Eigen::VectorXd& values) {
  return Move(values[0]);
}

bool Motor::Stop() {
  return Move(0.0);
}

bool Motor::Move(double value) {
  env_.lock()->SetMotorValue(id_, value);
  return true;
}

}  // namespace mujoco
}  // namespace huron
