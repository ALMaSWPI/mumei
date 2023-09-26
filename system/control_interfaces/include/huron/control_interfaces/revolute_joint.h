#pragma once

#include <memory>

#include "joint.h"

namespace huron {

/**
 * Represents a revolute joint.
 *
 * A revolute joint contains a rotational motor and an encoder. The encoder
 * may be mounted on an axis different from the motor axis. The joint can
 * have gears, which may make encoder measurements not equal to joint
 * movements. Two gear ratios are introduced to account for this reduction.
 *
 * By convention, gear ratios mentioned in this project refers to the ratio
 * of output torque over input torque.
 *
 * @ingroup control_interfaces
 */
class RevoluteJoint : public Joint {
 public:
  /**
   * Constructs a revolute joint.
   *
   * @param motor unique_ptr of a @see Motor.
   * @param encoder unique_ptr of a @see RotaryEncoder.
   * @param gear_ratio_1 Reduction from motor axis to encoder axis.
   * @param gear_ratio_2 Reduction from encoder axis to joint axis.
   */
  explicit RevoluteJoint(std::unique_ptr<Motor> motor,
                         std::unique_ptr<RotaryEncoder> encoder,
                         float gear_ratio_1, float gear_ratio_2);
  RevoluteJoint(const RevoluteJoint&) = delete;
  RevoluteJoint& operator=(const RevoluteJoint&) = delete;
  virtual ~RevoluteJoint() = default;

  /**
   * Gets the joint angle in radians.
   */
  float GetPosition() override;

  /**
   * Gets the joint velocity in radians/s.
   */
  float GetVelocity() override;

  /**
   * Gets the joint acceleration in radians/s^2.
   */
  float GetAcceleration() override;

 private:
  float gear_ratio_1_ = 1.0;
  float gear_ratio_2_ = 1.0;
};

}  // namespace huron
