#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "mumei/utils/time.h"
#include "mumei/locomotion/zero_moment_point_ft_sensor.h"
#include "mumei/control_interfaces/legged_robot.h"
#include "ros2_driver/ros_env.h"
#include "ros2_driver/force_torque_sensor.h"
#include "ros2_driver/joint_state_provider.h"
#include "ros2_driver/joint_group_controller.h"

using namespace mumei;  //NOLINT

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
mumei::ros2::Ros2Environment env(
  std::bind(loop));

// Create a robot
mumei::LeggedRobot robot;

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
  auto root_joint_sp = env.CreateJointStateProvider("root_jsp",
                                                    "/p3d/odom", 0, 7, 0, 6,
                                                    true);

  std::vector<std::shared_ptr<mumei::StateProvider>> joint_sp_list;
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
  std::vector<std::shared_ptr<mumei::ForceTorqueSensor>> ft_sensor_list;
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
  // while (rclcpp::ok()) {
  robot.UpdateAllStates();
  // Prints out joint states
  Eigen::VectorXd joint_positions = robot.GetJointPositions();
  Eigen::VectorXd joint_velocities = robot.GetJointVelocities();
  std::cout << "Positions:\n" << joint_positions.transpose() << std::endl;
  std::cout << "Velocities:\n" << joint_velocities.transpose() << std::endl;
  std::cout << "\n\n";

  // After 10 seconds, move the joints
  if (since(start).count() > 10000 && !moved) {
    moved = true;
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(12);
    cmd << 0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 10.0, 0.0, 0.0;
    robot.Move(cmd);
  }
}
