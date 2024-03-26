#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "huron/utils/time.h"
#include "huron/locomotion/zero_moment_point_ft_sensor.h"
#include "huron_ros2/huron.h"
#include "huron_ros2/force_torque_sensor.h"
#include "huron_ros2/joint_state_provider.h"
#include "huron_ros2/joint_group_controller.h"

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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto huron_node = std::make_shared<ros2::HuronNode>();
  // Instantiate a Huron object
  ros2::Huron robot(huron_node);
  robot.GetModel()->AddModelImpl(multibody::ModelImplType::kPinocchio);
  robot.GetModel()->BuildFromUrdf(
      ament_index_cpp::get_package_share_directory("huron_description") +
        "/urdf/huron.urdf",
      multibody::JointType::kFreeFlyer);

  auto root_joint_sp = std::make_shared<ros2::JointStateProvider>(
        0, 7,
        0, 6);
  robot.RegisterStateProvider(root_joint_sp, true);
  robot.GetModel()->SetJointStateProvider(1, root_joint_sp);
  std::vector<std::weak_ptr<ros2::JointStateProvider>> joint_sp_list;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    // Prints joint names in the internal model order
    auto joint = robot.GetModel()->GetJoint(i);
    std::cout << "Joint " << i
              << ", name = " << joint->Info()->name()
              << ", id_q = " << joint->id_q()
              << ", nq = " << joint->nq()
              << ", id_v = " << joint->id_v()
              << ", nv = " << joint->nv()
              << std::endl;
    auto joint_sp = std::make_shared<ros2::JointStateProvider>(
        i + 7, 1,
        i + 6, 1);
    joint_sp_list.push_back(joint_sp);
    robot.RegisterStateProvider(joint_sp, true);
    robot.GetModel()->SetJointStateProvider(
      robot.GetModel()->GetJointIndex(joint_names[i]),
      joint_sp);
  }

  robot.GetModel()->Finalize();

  // Register force torque sensors
  std::shared_ptr<huron::ros2::ForceTorqueSensor> l_ft_sensor =
    std::make_shared<ros2::ForceTorqueSensor>(
      false,  // reverse wrench direction
      robot.GetModel()->GetFrame("l_ankle_roll_joint"));
  robot.RegisterStateProvider(l_ft_sensor);
  std::shared_ptr<huron::ros2::ForceTorqueSensor> r_ft_sensor =
    std::make_shared<ros2::ForceTorqueSensor>(
      false,  // reverse wrench direction
      robot.GetModel()->GetFrame("r_ankle_roll_joint"));
  robot.RegisterStateProvider(r_ft_sensor);

  // Initialize ZMP
  std::vector<std::shared_ptr<ForceTorqueSensor>> ft_sensor_list;
  ft_sensor_list.push_back(l_ft_sensor);
  ft_sensor_list.push_back(r_ft_sensor);
  std::shared_ptr<ZeroMomentPoint> zmp =
    std::make_shared<ZeroMomentPointFTSensor>(
      robot.GetModel()->GetFrame("universe"),
      0.005,
      ft_sensor_list);
  robot.InitializeZmp(zmp);

  // Register joint group controller
  auto jgc = std::make_shared<ros2::JointGroupController>(12);
  robot.AddToGroup(jgc);

  // Add components to ROS2 node
  huron_node->AddJointStateProvider(root_joint_sp, "/p3d/odom", 7, 6, true);
  for (auto& joint_sp : joint_sp_list) {
    huron_node->AddJointStateProvider(joint_sp.lock(), "joint_states", 1, 1);
  }
  huron_node->AddForceTorqueSensor(l_ft_sensor, "huron/sensor/l1_ft_sensor");
  huron_node->AddForceTorqueSensor(r_ft_sensor, "huron/sensor/r1_ft_sensor");
  huron_node->AddJointGroupController(jgc,
                                      "joint_group_effort_controller/commands");
  huron_node->Finalize();

  auto start = std::chrono::steady_clock::now();
  bool moved = false;

  while (rclcpp::ok()) {
    robot.Loop();
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
  rclcpp::shutdown();

  return 0;
}
