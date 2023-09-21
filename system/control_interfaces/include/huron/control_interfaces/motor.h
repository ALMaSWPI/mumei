#pragma once

#include "moving_component.h"

namespace huron {

class Motor : public MovingComponent{
public:
    float GetDesiredValue();
    virtual bool Stop() = 0;
    virtual bool Move(float value) = 0;
    virtual bool HasReachGoal() = 0;

private:
    float desired_value_;

};

}
