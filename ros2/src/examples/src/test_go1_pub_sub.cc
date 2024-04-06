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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto huron_node = std::make_shared<ros2::HuronNode>();
  // Instantiate a Huron object
  ros2::Huron robot(huron_node);
  robot.GetModel()->AddModelImpl(multibody::ModelImplType::kPinocchio);
  robot.GetModel()->BuildFromUrdf(
      ament_index_cpp::get_package_share_directory("go1_description") +
        "/urdf/go1.urdf",
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

  // Register joint group controller
  auto jgc = std::make_shared<ros2::JointGroupController>(12);
  robot.AddToGroup(jgc);

  // Add components to ROS2 node
  huron_node->AddJointStateProvider(root_joint_sp, "/p3d/odom", 7, 6, true);
  for (auto& joint_sp : joint_sp_list) {
    huron_node->AddJointStateProvider(joint_sp.lock(), "joint_states", 1, 1);
  }
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
      cmd << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
      robot.Move(cmd);
    }
  }
  rclcpp::shutdown();

  return 0;
}
