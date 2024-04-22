#include <iostream>
#include <cstdio>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "huron/utils/time.h"

#include "huron_mujoco/huron.h"
#include "huron_mujoco/mujoco_env.h"

using namespace huron;  //NOLINT

const std::vector<std::string> joints_list = {
  "x",
};

void setup();
void loop();

// Create an environment
huron::mujoco::MujocoEnvironment env(
  std::bind(loop));

// Create a robot
huron::mujoco::Huron robot;

// Misc variables
auto start = std::chrono::steady_clock::now();
bool moved = false;

int main(int argc, char* argv[]) {
  env.Initialize(argc, argv);

  setup();
  env.Finalize();

  start = std::chrono::steady_clock::now();

  env.Loop();

  env.Exit();
  return 0;
}

void setup() {
  // Environment setup
  auto enc_x = env.CreateEncoder("x");
  auto enc_theta = env.CreateEncoder("theta");

  robot.GetModel()->AddModelImpl(multibody::ModelImplType::kPinocchio);
  robot.GetModel()->BuildFromUrdf(
      "cartpole.urdf",
      multibody::JointType::kFixed);  // joint type doesn't matter here

  robot.RegisterStateProvider(enc_x, true);
  robot.GetModel()->SetJointStateProvider(
      robot.GetModel()->GetJointIndex("x"), enc_x);
  robot.RegisterStateProvider(enc_theta, true);
  robot.GetModel()->SetJointStateProvider(
      robot.GetModel()->GetJointIndex("theta"), enc_theta);

  robot.GetModel()->Finalize();

  auto motor_x = env.CreateMotor("x");
  robot.AddToGroup(motor_x);
}

void loop() {
  robot.UpdateAllStates();
  robot.GetModel()->ForwardKinematics();

  Eigen::VectorXd joint_positions = robot.GetJointPositions();
  std::cout << "Positions:\n" << joint_positions.transpose() << std::endl;
  Eigen::VectorXd joint_velocities = robot.GetJointVelocities();
  std::cout << "Velocities:\n" << joint_velocities.transpose() << std::endl;

  // After 3 seconds, move the joints
  // if (since(start).count() > 3000 && !moved) {
  //   moved = true;
  // }
  // if (moved) {
  //   Eigen::VectorXd cmd = Eigen::VectorXd::Zero(12);
  //   cmd << 0.0, 0.0, torque(2, 0), torque(1, 0), torque(0, 0), 0.0,
  //          0.0, 0.0, torque(2, 0), torque(1, 0), torque(0, 0), 0.0;
  //   robot.Move(cmd);
  // }
}
