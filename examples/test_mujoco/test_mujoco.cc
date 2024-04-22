#include <iostream>
#include "huron/utils/time.h"

#include "huron_mujoco/huron.h"
#include "huron_mujoco/mujoco_env.h"

using namespace huron;  //NOLINT

void setup(std::string urdf_path);
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
  // Usage: ./test_mujoco <path_to_xml> <path_to_urdf>
  env.Initialize(argc, argv);
  std::string urdf_path = std::string(argv[2]);
  std::cout << "URDF path: " << urdf_path << std::endl;

  setup(urdf_path);
  env.Finalize();

  start = std::chrono::steady_clock::now();

  env.Loop();

  env.Exit();
  return 0;
}

void setup(std::string urdf_path) {
  // Environment setup
  robot.GetModel()->AddModelImpl(multibody::ModelImplType::kPinocchio);
  robot.GetModel()->BuildFromUrdf(
      urdf_path,
      multibody::JointType::kFreeFlyer);  // joint type doesn't matter here

  // floating base
  auto floating_base = env.CreateFloatingBase();
  robot.RegisterStateProvider(floating_base, true);
  robot.GetModel()->SetJointStateProvider(1, floating_base);

  for (int i = 2; i < robot.GetModel()->num_joints(); ++i) {
    std::string joint_name = robot.GetModel()->GetJoint(i)->Info()->name();
    std::cout << "Joint idx: " << i << "\t" << joint_name << std::endl;
    auto enc = env.CreateEncoder(joint_name);
    robot.RegisterStateProvider(enc, true);
    robot.GetModel()->SetJointStateProvider(i, enc);
  }

  robot.GetModel()->Finalize();

  // minus (first) dummy joint and floating base
  for (int i = 0; i < robot.GetModel()->num_joints() - 2; ++i) {
    auto motor_x = env.CreateMotor(i);
    robot.AddToGroup(motor_x);
  }
}

void loop() {
  robot.UpdateAllStates();
  robot.GetModel()->ForwardKinematics();

  // env.PrintStates();

  Eigen::VectorXd joint_positions = robot.GetJointPositions();
  // std::cout << "Positions:\n" << joint_positions.transpose() << std::endl;
  Eigen::VectorXd joint_velocities = robot.GetJointVelocities();
  // std::cout << "Velocities:\n" << joint_velocities.transpose() << std::endl;

  // After 3 seconds, move the joints
  if (since(start).count() > 3000 && !moved) {
    moved = true;
  }
  if (moved) {
    Eigen::VectorXd cmd = 100 * Eigen::VectorXd::Random(
        robot.GetModel()->num_joints() - 2);
    std::cout << "Command:\n" << cmd.transpose() << std::endl;
    robot.Move(cmd);
  }
}
