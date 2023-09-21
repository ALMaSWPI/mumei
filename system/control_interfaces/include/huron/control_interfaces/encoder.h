#pragma once

#include <math.h>
#include "generic_component.h"

namespace huron {

class Encoder : public GenericComponent {
private:
    float count_;
    float prev_count_;
    float cpr_;

public:
    void  Reset() {
        count_ = 0.0;
        prev_count_ = 0.0;
    }
    virtual float GetCount() = 0;
    virtual float GetPrevCount() = 0;
    virtual float GetCPR() = 0;
    virtual float GetAngleRadian() = 0;
    virtual float GetAngleDegree() = 0;

    virtual float GetVelocity() = 0;
    float GetVelocityRad() {
        return GetVelocity() / cpr_ * 2 * M_PIf;
    }
};

}
