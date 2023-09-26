#pragma once

#include <cmath>

#include "generic_component.h"

namespace huron {

/**
 * Abstract class for using an encoder.
 *
 * @ingroup control_interfaces
 */
class Encoder : public GenericComponent {
 protected:
  float count_ = 0;
  float prev_count_ = 0;
  float velocity_ = 0;
  float prev_velocity_ = 0;
  float cpr_;

 public:
  explicit Encoder(float cpr) : cpr_(cpr) {}
  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  virtual ~Encoder() = default;

  /**
     * Resets the encoder count.
     */
  void Reset() {
    count_ = 0.0;
    prev_count_ = 0.0;
  }

  /**
     * Gets the current encoder count.
     */
  virtual float GetCount() = 0;

  /**
     * Gets the previous encoder count.
     */
  float GetPrevCount() {
    return prev_count_;
  }

  /**
     * Gets the counts per revolution (CPR).
     */
  float GetCPR() {
    return cpr_;
  }

  /**
     * Gets the current angle in radians.
     */
  float GetAngleRadian() {
    return GetCount() / cpr_ * 2.0 * M_PI;
  }

  /**
     * Gets the current angle in degrees.
     */
  float GetAngleDegree() {
    return GetCount() / cpr_ * 360.0;
  }

  /**
     * Gets the current velocity in counts/second.
     */
  virtual float GetVelocity() = 0;

  /**
     * Gets the current velocity in radians/second.
     */
  float GetVelocityRadian() {
    return GetVelocity() / cpr_ * 2 * M_PI;
  }

  /**
     * Gets the current velocity in degrees/second.
     */
  float GetVelocityDegree() {
    return GetVelocity() / cpr_ * 360.0;
  }
};

}  // namespace huron
