#pragma once

#include "moving_component.h"

namespace huron {

  class Motor : public MovingComponent {

   public:
    Motor();
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;
    ~Motor() = default;

    float GetDesiredValue() {
      return desired_value_;
    }
    virtual bool HasReachGoal() = 0;

   private:
    float desired_value_;
  };

}// namespace huron
