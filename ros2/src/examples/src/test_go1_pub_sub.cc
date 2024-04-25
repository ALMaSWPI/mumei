#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "huron/utils/time.h"
#include "huron/locomotion/zero_moment_point_ft_sensor.h"
#include "huron/control_interfaces/legged_robot.h"
#include "huron_ros2/ros_env.h"
#include "huron_ros2/force_torque_sensor.h"
#include "huron_ros2/joint_state_provider.h"
#include "huron_ros2/joint_group_controller.h"

using namespace huron;  //NOLINT

// Joint names order from ROS2
const std::array<std::string, 12> joint_names = {
  "FR_calf_joint",
  "FL_thigh_joint",
  "FL_calf_joint",
  "RL_hip_joint",
  "FL_hip_joint",
  "RL_calf_joint",
  "FR_hip_joint",
  "FR_thigh_joint",
  "RR_hip_joint",
  "RL_thigh_joint",
  "RR_thigh_joint",
  "RR_calf_joint"
};

void setup();
void loop();

// Create an environment
huron::ros2::Ros2Environment env(
  std::bind(loop));

// Create a robot
huron::LeggedRobot robot;

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
  auto root_joint_sp = env.CreateJointStateProvider("/p3d/odom", 0, 7, 0, 6,
                                                    true);

  std::vector<std::shared_ptr<huron::StateProvider>> joint_sp_list;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    joint_sp_list.push_back(env.CreateJointStateProvider(
        "joint_states",
        i + 7, 1,
        i + 6, 1));
  }

  robot.GetModel()->AddModelImpl(multibody::ModelImplType::kPinocchio);
  robot.GetModel()->BuildFromUrdf(
      ament_index_cpp::get_package_share_directory("go1_description") +
        "/urdf/go1.urdf",
      multibody::JointType::kFreeFlyer);

  // Root joint
  robot.RegisterStateProvider(root_joint_sp, true);
  robot.GetModel()->SetJointStateProvider(1, root_joint_sp);

  // 12 joints
  for (size_t i = 0; i < joint_names.size(); ++i) {
    robot.RegisterStateProvider(joint_sp_list[i], true);
    robot.GetModel()->SetJointStateProvider(
      robot.GetModel()->GetJointIndex(joint_names[i]), joint_sp_list[i]);
  }

  robot.GetModel()->Finalize();

  // Register joint group controller
  auto jgc = env.CreateJointGroupController(
      "joint_group_effort_controller/commands", 12);

  robot.AddToGroup(jgc);
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
