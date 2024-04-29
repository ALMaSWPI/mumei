#include "mumei/mujoco/encoder.h"
#include "mumei/mujoco/mujoco_env.h"

namespace mumei {
namespace mujoco {

Encoder::Encoder(const std::string& name, int mj_id,
             std::weak_ptr<MujocoEnvironment> env, double gear_ratio)
    : mumei::Encoder(name, gear_ratio), mj_id_(mj_id), env_(env) {}

void Encoder::Initialize() {
}

void Encoder::SetUp() {
}

void Encoder::Terminate() {
}

void Encoder::RequestStateUpdate() {
  Eigen::Vector2d tmp = env_.lock()->GetEncoderValues(mj_id_);
  position_ = tmp(0);
  velocity_ = tmp(1);
}

double Encoder::GetPosition() const {
  return position_;
}

double Encoder::GetVelocity() const {
  return velocity_;
}

void Encoder::Reset() {
  position_ = 0.0;
  velocity_ = 0.0;
}


}  // namespace mujoco
}  // namespace mumei
