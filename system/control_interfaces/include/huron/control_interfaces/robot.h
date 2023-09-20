#include "control_interfaces/Limb.h"

#ifndef __CI_ROBOT_H
#define __CI_ROBOT_H

class Robot {
public:
    void Init(Limb limb_list[]);
private:
    Limb limbs_;

};
#endif
