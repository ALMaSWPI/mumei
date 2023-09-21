#include "joint.h"

void Joint::Init(JointType joint_type, Motor motor, Encoder encoder)
{
  joint_type_ = joint_type;
  motor_ = motor;
  encoder_ = encoder;
}

void Joint::Init(JointType joint_type, Motor motor, Encoder encoder,
                 int gear_ratio_1, int gear_ratio_2,
                 float position, float velocity, float acceleration)
{
  joint_type_ = joint_type;
  motor_ = motor;
  encoder_ = encoder;
  gear_ratio_1_ = 1;
  gear_ratio_2_ = 1;
  position_ = position;
  velocity_ = velocity;
  acceleration_ = acceleration;
}

void Joint::Stop()
{
  motor_.Stop();
}

float Joint::Move(float value){
  motor_.Move(value);
}

float Joint::GetPosition(){
  return encoder_.GetAngleRadian()/gear_ratio_1_/gear_ratio_2_;
}

float Joint::GetVelocity(){
  return encoder_.GetVelocityRad()/gear_ratio_1_/gear_ratio_2_;
}

float Joint::GetAcceleration(){
  //Update later
  return -1.0;
}