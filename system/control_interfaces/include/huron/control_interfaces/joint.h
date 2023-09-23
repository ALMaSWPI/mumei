#pragma once

#include <memory>

#include "encoder.h"
#include "motor.h"
#include "moving_component.h"

namespace huron {

/**
 * Abstract class representing a joint.
 *
 * Currently, the structure supports 1-DoF joints.
 *
 * @ingroup control_interfaces
 */
class Joint : public MovingComponent {
 public:
  explicit Joint(std::unique_ptr<Motor> motor,
                 std::unique_ptr<Encoder> encoder);
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;
  ~Joint() = default;

  void Configure() override;
  void Initialize() override;
  void SetUp() override;
  void Terminate() override;

  bool Move(float value) override;
  bool Move(const std::vector<float>& value) override;
  bool Stop() override;

  /**
   * Gets the position of the joint.
   */
  virtual float GetPosition() = 0;
  /**
   * Gets the velocity of the joint.
   */
  virtual float GetVelocity() = 0;
  /**
   * Gets the acceleration of the joint.
   */
  virtual float GetAcceleration() = 0;

 protected:
  std::unique_ptr<Motor> motor_;
  std::unique_ptr<Encoder> encoder_;
};

}// namespace huron
