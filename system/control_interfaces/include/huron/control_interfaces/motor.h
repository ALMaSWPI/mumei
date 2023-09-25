#pragma once

#include <vector>

#include "moving_component.h"

namespace huron {

/**
 * Abstract class for a motor.
 *
 * A motor takes in input value(s) and actuates accordingly. It does not
 * need to know its position or velocity; that is the job of encoders
 * and other sensors.
 *
 * @ingroup control_interfaces
 */
class Motor : public MovingComponent {
 public:
  Motor() = default;
  Motor(const Motor&) = delete;
  Motor& operator=(const Motor&) = delete;
  ~Motor() = default;
};

}  // namespace huron
