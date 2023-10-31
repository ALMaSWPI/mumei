#pragma once

#include <vector>

#include "generic_component.h"

namespace huron {

class MovingGroupComponent : public GenericComponent {
 public:
  explicit MovingGroupComponent(std::unique_ptr<Configuration> config)
    : GenericComponent(std::move(config)) {}
  MovingGroupComponent(const MovingGroupComponent&) = delete;
  MovingGroupComponent& operator=(const MovingGroupComponent&) = delete;
  ~MovingGroupComponent() = default;

  virtual bool Move(const std::vector<double>& values) = 0;

  virtual bool Stop() = 0;
};

}  // namespace huron
