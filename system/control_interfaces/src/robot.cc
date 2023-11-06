#include "huron/control_interfaces/robot.h"

namespace huron {

Robot::Robot(std::unique_ptr<RobotConfiguration> config)
  : MovingGroupComponent(std::move(config)) {}

Robot::Robot()
  : Robot::Robot(std::make_unique<RobotConfiguration>()) {}

std::vector<double> Robot::GetJointPosition() {
  std::vector<double> result;
  for (auto joint : joints_) {
    result.push_back(joint->GetPosition());
  }
  return result;
}

std::vector<double> Robot::GetJointVelocity() {
  std::vector<double> result;
  for (auto joint : joints_) {
    result.push_back(joint->GetVelocity());
  }
  return result;
}

bool Robot::AddJoint(std::shared_ptr<Joint> joint) {
  joints_.push_back(std::move(joint));
  return true;
}

void Robot::Initialize() {
  for (auto joint : joints_) {
    joint->Initialize();
  }
}

void Robot::SetUp() {
  for (auto joint : joints_) {
    joint->SetUp();
  }
}

void Robot::Terminate() {
  for (auto joint : joints_) {
    joint->Terminate();
  }
}

bool Robot::Move(const std::vector<double>& values) {
  for (size_t i = 0; i < values.size(); ++i) {
    joints_[i]->Move(values[i]);
  }
  return true;
}

bool Robot::Stop() {
  return Move(std::vector<double>(joints_.size()));
}

}  // namespace huron
