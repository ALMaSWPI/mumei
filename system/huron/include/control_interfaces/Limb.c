#include "Limb.h"

void Limb::Init(Joint joint_list[]){
    for (std::size_t i = 0; auto& d : joint_list) {
        joints_.push_back(d);
    }
}

void Limb::AddJoint(Joint joint){
    joints_.push_back(joint);
}

bool Limb::Move(float values[]){
    for (std::size_t i = 0; auto& d : joints_) {
        d.motor_.Move(value[i]);
    }
    return True;
}

bool Limb::Stop(){
    for (std::size_t i = 0; auto& d : joints_) {
        d.motor_.Stop();
    }
    return True;
}