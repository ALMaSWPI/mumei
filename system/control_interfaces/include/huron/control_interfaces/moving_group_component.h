#pragma once

#include <vector>

#include "generic_component.h"

namespace huron {

  class MovingGroupComponent : public GenericComponent {
    virtual bool Move(std::vector<float> vals) = 0;

    virtual bool Stop() = 0;
  };

}// namespace huron
