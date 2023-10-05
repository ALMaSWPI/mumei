#include "huron/control_interfaces/revolute_joint.h"

namespace huron {

RevoluteJoint::RevoluteJoint(std::unique_ptr<Motor> motor,
                             std::unique_ptr<RotaryEncoder> encoder,
                             std::unique_ptr<RevoluteJointConfiguration> conf)
    : Joint(std::move(motor), std::move(encoder), std::move(conf)) {
  gear_ratio_1_ = std::any_cast<float>(conf.get()->Get("gear_ratio_1"));
  gear_ratio_2_ = std::any_cast<float>(conf.get()->Get("gear_ratio_2"));
}

RevoluteJoint::RevoluteJoint(std::unique_ptr<Motor> motor,
                             std::unique_ptr<RotaryEncoder> encoder,
                             float gear_ratio_1, float gear_ratio_2)
    : RevoluteJoint(
        std::move(motor),
        std::move(encoder),
        std::make_unique<RevoluteJointConfiguration>(
          gear_ratio_1, gear_ratio_2)) {}


RevoluteJoint::RevoluteJoint(std::unique_ptr<Motor> motor,
                             std::unique_ptr<RotaryEncoder> encoder)
    : RevoluteJoint(std::move(motor), std::move(encoder), 
                    std::make_unique<RevoluteJointConfiguration>()) {}

float RevoluteJoint::GetPosition() {
  return encoder_->GetPosition() / gear_ratio_2_;
}

float RevoluteJoint::GetVelocity() {
  return encoder_->GetVelocity() / gear_ratio_2_;
}

float RevoluteJoint::GetAcceleration() {
  // Update later
  return -1.0;
}

}  // namespace huron
