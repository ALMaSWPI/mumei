#ifndef __HURON_TORQUE_MOTOR_H
#define __HURON_TORQUE_MOTOR_H

#include "control_interfaces/Motor.h"

class TorqueMotor : Motor(){
public:
    void Init();

private:
    float current_limit_ = 0;
    float velocity_limit_ = 0;
    float desired_value_ = 0;

}

#endif