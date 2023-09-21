#pragma once

#include <memory>
#include "moving_component.h"
#include "motor.h"
#include "encoder.h"

namespace huron {

class Joint : public MovingComponent{
public:
  explicit Joint(std::unique_ptr<Motor> motor,
                 std::unique_ptr<Encoder> encoder);
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;
  ~Joint() = default;

  bool Move(float value) override;
  bool Stop() override;
  virtual float GetPosition() = 0;
  virtual float GetVelocity() = 0;
  virtual float GetAcceleration() = 0;

protected:
  std::unique_ptr<Motor> motor_;
  std::unique_ptr<Encoder> encoder_;
};

}
