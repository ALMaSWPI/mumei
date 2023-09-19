#include "control_interfaces/GenericComponent.h"

#ifndef __CI_MOVING_COMPONENT_H
#define __CI_MOVING_COMPONENT_H

class MovingComponent: GenericComponent{
    virtual bool move(){} =0;
    
    virtual bool stop(){} = 0;
}
#endif