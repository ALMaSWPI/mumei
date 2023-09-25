#pragma once

#include <vector>

#include "joint.h"
#include "moving_group_component.h"

namespace huron {

class Limb : public MovingGroupComponent {
 public:
  void Init(std::vector<Joint> joints);
  void AddJoint(Joint& joint);

 private:
  std::vector<Joint> joints_;
};

}  // namespace huron
