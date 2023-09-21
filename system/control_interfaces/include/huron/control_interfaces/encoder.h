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
    Encoder();
    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;
    ~Encoder() = default;

    void  Reset() {
        count_ = 0.0;
        prev_count_ = 0.0;
    }

    virtual float GetCount() = 0;

    float GetPrevCount() {
        return prev_count_;
    }

    float GetCPR() {
        return cpr_;
    }

    float GetAngleRadian() {
        return count_ / cpr_ * 2.0 * M_PI;
    }

    float GetAngleDegree() {
      return count_ / cpr_ * 360.0;
    }

    virtual float GetVelocity() = 0;

    float GetVelocityRadian() {
        return GetVelocity() / cpr_ * 2 * M_PI;
    }
};

}
