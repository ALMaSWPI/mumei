#include "control_interfaces/MovingComponent.h"

class Motor : MovingComponent{
    public:
    float GetDesiredValue();
    virtual bool Stop() = 0;
    virtual bool Move(float value) = 0;
    virtual bool SetUp() = 0;
    virtual bool HasReachGoal() = 0;
    
    private: 
    float desired_value;

};