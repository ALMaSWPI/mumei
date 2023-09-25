#pragma once

#include <vector>

#include "generic_component.h"

namespace huron {

class MovingGroupComponent : public GenericComponent {
 public:
  virtual bool Move(std::vector<float> values) = 0;
  virtual bool Move(const std::vector<std::vector<float>>& values) = 0;

  virtual bool Stop() = 0;
};

}  // namespace huron
