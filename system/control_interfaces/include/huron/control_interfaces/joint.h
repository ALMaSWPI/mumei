#pragma once

#include <memory>
#include "moving_component.h"
#include "motor.h"
#include "encoder.h"

namespace huron {

class Joint : public MovingComponent{
  enum JointType{
    REVOLUTE = 1,
    PRISMATIC = 2
  };

public:
  float GetPosition();
  float GetVelocity();
  float GetAcceleration();

private:
  JointType joint_type_;
  std::unique_ptr<Motor> motor_;
  std::unique_ptr<Encoder> encoder_;
  float gear_ratio_1_ = 1.0;
  float gear_ratio_2_ = 1.0;

  float position_ = 0.0;
  float velocity_ = 0.0;
  float acceleration_ = 0.0;
};

}
