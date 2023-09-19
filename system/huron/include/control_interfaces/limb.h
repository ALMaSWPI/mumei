#include "control_interfaces/Joint.h"
#include "control_interfaces/MovingComponent.h"

#ifndef __CI_LIMB_H
#define __CI_LIMB_H

class Limb : MovingComponent {
public:
    void Init(Joint joint_list[]);
    void AddJoint(Joint joint);
    bool Move(float values[]);
    bool Stop();

private:
    std::vector<Joint> joints_[];

};
#endif