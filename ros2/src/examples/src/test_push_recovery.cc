#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "huron/utils/time.h"
#include "huron/locomotion/zero_moment_point_ft_sensor.h"
#include "huron/control_interfaces/legged_robot.h"
#include "huron_ros2/ros_env.h"
#include "huron_ros2/force_torque_sensor.h"
#include "huron_ros2/joint_state_provider.h"
#include "huron_ros2/joint_group_controller.h"
#include "huron/control/push_recovery.h"

using namespace huron;  //NOLINT

// Joint names order from ROS2
const std::array<std::string, 12> joint_names = {
  "l_hip_yaw_joint",
  "l_hip_roll_joint",
  "l_knee_pitch_joint",
  "l_ankle_pitch_joint",
  "r_hip_roll_joint",
  "r_hip_pitch_joint",
  "r_knee_pitch_joint",
  "l_hip_pitch_joint",
  "l_ankle_roll_joint",
  "r_ankle_pitch_joint",
  "r_hip_yaw_joint",
  "r_ankle_roll_joint"
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

PushRecoveryControl controller;

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
  auto root_joint_sp = env.CreateJointStateProvider("root_jsp",
                                                    "/p3d/odom", 0, 7, 0, 6,
                                                    true);

  std::vector<std::shared_ptr<huron::StateProvider>> joint_sp_list;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    joint_sp_list.push_back(env.CreateJointStateProvider(
        "jsp_" + std::to_string(i),
        "joint_states",
        i + 7, 1,
        i + 6, 1));
  }

  robot.GetModel()->AddModelImpl(multibody::ModelImplType::kPinocchio);
  robot.GetModel()->BuildFromUrdf(
      ament_index_cpp::get_package_share_directory("huron_description") +
        "/urdf/huron.urdf",
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

  // Register force torque sensors
  auto l_ft_sensor = env.CreateForceTorqueSensor(
      "l_ft_sensor",
      "huron/sensor/l1_ft_sensor",
      false,  // reverse wrench direction
      robot.GetModel()->GetFrame("l_ankle_roll_joint"));
  auto r_ft_sensor = env.CreateForceTorqueSensor(
      "r_ft_sensor",
      "huron/sensor/r1_ft_sensor",
      false,  // reverse wrench direction
      robot.GetModel()->GetFrame("r_ankle_roll_joint"));

  // Register joint group controller
  auto jgc = env.CreateJointGroupController(
      "joint_group_effort_controller/commands", 12);

  // Robot setup
  robot.RegisterStateProvider(l_ft_sensor);
  robot.RegisterStateProvider(r_ft_sensor);

  // Initialize ZMP
  std::vector<std::shared_ptr<huron::ForceTorqueSensor>> ft_sensor_list;
  ft_sensor_list.push_back(l_ft_sensor);
  ft_sensor_list.push_back(r_ft_sensor);
  std::shared_ptr<ZeroMomentPoint> zmp =
    std::make_shared<ZeroMomentPointFTSensor>(
      robot.GetModel()->GetFrame("universe"),
      0.005,
      ft_sensor_list);
  robot.InitializeZmp(zmp);

  robot.AddToGroup(jgc);
}

void loop() {
  robot.UpdateAllStates();
  robot.GetModel()->ForwardKinematics();
  Eigen::Vector3d com = robot.GetModel()->EvalCenterOfMassPosition();
  std::cout << "CoM: " << com.transpose() << std::endl;

  Eigen::VectorXd joint_positions = robot.GetJointPositions();
  std::cout << "Positions:\n" << joint_positions.transpose() << std::endl;
  Eigen::VectorXd joint_velocities = robot.GetJointVelocities();
  std::cout << "Velocities:\n" << joint_velocities.transpose() << std::endl;

  // After 3 seconds, move the joints
  if (since(start).count() > 3000 && !moved) {
    moved = true;
  }
  if (moved) {
    std::cout << "Calculating new torque\n";

    Eigen::Vector2d cop = robot.EvalZeroMomentPoint();
    std::cout << "CoP: " << cop.transpose() << std::endl;

    Eigen::MatrixXd torque = controller.GetTorque(
        cop, joint_positions, joint_velocities);

    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(12);
    cmd << 0.0, 0.0, torque(2, 0), torque(1, 0), torque(0, 0), 0.0,
           0.0, 0.0, torque(2, 0), torque(1, 0), torque(0, 0), 0.0;
    robot.Move(cmd);
  }
}
