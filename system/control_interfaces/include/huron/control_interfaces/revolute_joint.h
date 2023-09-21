#pragma once

#include "joint.h"

namespace huron {

class RevoluteJoint : public Joint {

public:
  explicit RevoluteJoint(std::unique_ptr<Motor> motor,
                         std::unique_ptr<Encoder> encoder,
                         float gear_ratio_1, float gear_ratio_2);
  float GetPosition() override;
  float GetVelocity() override;
  float GetAcceleration() override;

private:
  float gear_ratio_1_ = 1.0;
  float gear_ratio_2_ = 1.0;
};

}
