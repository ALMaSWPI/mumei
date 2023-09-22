#include "robot.h"

void Robot::Init(Limb limb_list[]) {
  for (std::size_t i = 0; auto& d: limb_list) {
    limbs_.push_back(d);
  }
}