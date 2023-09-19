#include "control_interfaces/MovingComponent.h"
#include "control_interfaces/Motor.h"
#include "control_interfaces/Encoder.h"

enum JointType{
    REVOLUTE = 1
    PRISMATIC = 2
}

class Joint : MovingComponent{
    
    public: 

        void Init();
        void Move(float value);
        void Stop();
        float GetPosition();
        float GetVelocity();
        float GetAcceleration();

    private:
        JointType joint_type_ = nullptr;
        Motor motor_ = nullptr;
        Encoder encoder_ = nullptr;
        gear_ratio_1_ = 1;
        gear_ratio_2_ = 1;

        float position_ = 0;
        float velocity_ = 0;
        float acceleration_ = 0;

}