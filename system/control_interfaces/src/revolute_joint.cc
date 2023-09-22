#include "huron/control_interfaces/revolute_joint.h"

namespace huron {

  RevoluteJoint::RevoluteJoint(std::unique_ptr<Motor> motor,
                               std::unique_ptr<Encoder> encoder,
                               float gear_ratio_1, float gear_ratio_2)
      : Joint(std::move(motor), std::move(encoder)),
        gear_ratio_1_(gear_ratio_1),
        gear_ratio_2_(gear_ratio_2) {}

  float RevoluteJoint::GetPosition() {
    return encoder_->GetAngleRadian() / gear_ratio_1_ / gear_ratio_2_;
  }

  float RevoluteJoint::GetVelocity() {
    return encoder_->GetVelocityRadian() / gear_ratio_1_ / gear_ratio_2_;
  }

  float RevoluteJoint::GetAcceleration() {
    //Update later
    return -1.0;
  }

}// namespace huron
