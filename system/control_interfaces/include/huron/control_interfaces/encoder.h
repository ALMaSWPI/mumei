#include "control_interfaces/GenericComponent.h"

#ifndef __CI_ENCODER_H
#define __CI_ENCODER_H

#include "control_interfaces/GenericComponent.h"
class Encoder : GenericComponent {
private:
    float count_;
    float prev_count_;
    float cpr_;

public:
    void  Reset();
    float GetCount();
    float GetPrevCount();
    float GetCPR();
    float GetAngleRadian();
    float GetAngleDegree();

    virtual float GetVelocity() = 0 ;
    float GetVelocityRad(); 

}
#endif