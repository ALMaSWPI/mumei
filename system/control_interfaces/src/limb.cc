#include "huron/control_interfaces/limb.h"
#include "huron/control_interfaces/joint.h"

namespace huron {

void Limb::Init(std::vector<Joint> joint_list) {
  for (int i =0; i < joint_list.size(); i++) {
    joints_.push_back(joint_list.at(i));
  }
}

void Limb::AddJoint(Joint joint) {
  joints_.push_back(joint);
}

bool Limb::Move(float values[]) {
  for (std::size_t i = 0; auto& d : joints_) {
    d.motor_.Move(value[i]);
  }
  return True;
}

bool Limb::Stop() {
  for (std::size_t i = 0; auto& d : joints_) {
    d.motor_.Stop();
  }
  return True;
}

}  // namespace huron
