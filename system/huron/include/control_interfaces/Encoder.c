#include "Encoder.h"
#include <cmath>


void Encoder::Reset(){
    count_ = 0;
    prev_count_ = 0;

};

float Encoder::GetCount(){
    return count_;
};

float Encoder::GetPrevCount(){
    return prev_count_;
};


float Encoder::GetCPR(){
    return cpr_;
};

float Encoder::GetAngleRadian(){
    return count_ / cpr_ * 2 * M_PI;
}

float Encoder::GetAngleDegree(){
    return count_ / cpr_ * 360;
}

float Encoder::GetVelocityRad(){
  return GetVelocity() / cpr_* 2 *M_PI; 
}