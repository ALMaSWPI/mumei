#pragma once
#include "generic_component.h"

namespace huron {

/**
 * Abstract class for encoder
 * A generic encoder has count and velocity.
 *
 * @ingroup control_interface
 */
class Encoder : public GenericComponent {
 public:
  Encoder() = default;
  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  virtual ~Encoder() = default;

  virtual float GetPosition() = 0;
  virtual float GetVelocity() = 0;
  virtual void Reset() = 0;
};

}  // namespace huron
