#include "control_interfaces/GenericComponent.h"
class MovingComponent: GenericComponent{
    virtual bool move(){} =0;
    
    virtual bool stop(){} = 0;
}