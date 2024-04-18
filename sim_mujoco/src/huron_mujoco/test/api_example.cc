#include <iostream>
#include "huron_ros2/huron.h"
#include "huron/sim_mujoco/huron.h"
#include "huron/utils/time.h"
using namespace huron;  //NOLINT

const double kGearRatio = 1.0;
const double kCPR = 4096.0;

const std::array<std::string, 8> joints_without_encoder = {
  "l_hip_yaw_joint",
  "l_hip_roll_joint",
  "l_ankle_pitch_joint",
  "r_hip_roll_joint",
  "l_ankle_roll_joint",
  "r_ankle_pitch_joint",
  "r_hip_yaw_joint",
  "r_ankle_roll_joint"
};

int main(int argc, char* argv[]) {
  mujoco::Huron robot;

  robot.GetModel()->AddModelImpl(multibody::ModelImplType::kPinocchio);
  robot.GetModel()->BuildFromUrdf("sim_mujoco/src/huron_model/huron.urdf",
    multibody::JointType::kFreeFlyer);
  robot.BuildFromXml("sim_mujoco/src/huron_model/huron.xml");
  // Instantiate a Huron object


  return 0;
}

