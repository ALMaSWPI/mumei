#include "huron/control_interfaces/joint.h"

namespace huron {

Joint::Joint(std::unique_ptr<Motor> motor,
      std::unique_ptr<Encoder> encoder)
    : motor_(std::move(motor)), encoder_(std::move(encoder)) {}

bool Joint::Move(float value){
  return motor_->Move(value);
}

bool Joint::Stop()
{
  return motor_->Stop();
}

}
