#include "huron_ros2/ros_env.h"
#include "huron_ros2/joint_state_provider.h"
#include "huron_ros2/force_torque_sensor.h"
#include "huron_ros2/joint_group_controller.h"

namespace huron {
namespace ros2 {

Ros2Environment::Ros2Environment(std::function<void(void)> loop_func,
                                 std::function<void(void)> exit_func)
    : Environment(loop_func, exit_func) {}

void Ros2Environment::Initialize(int argc,
                           char* argv[]) {
  rclcpp::init(argc, argv);
  huron_node_ = std::make_shared<ros2::HuronNode>();
}

void Ros2Environment::Configure(void* config) {
}

void Ros2Environment::Finalize() {
  huron_node_->Finalize();
}

void Ros2Environment::LoopPrologue() {
  rclcpp::spin_some(huron_node_);
}

void Ros2Environment::LoopEpilogue() {
}

void Ros2Environment::Exit() {
  exit_func_();
  rclcpp::shutdown();
}

std::shared_ptr<huron::StateProvider> Ros2Environment::CreateJointStateProvider(
  const std::string& name,
  const std::string& topic,
  size_t id_q, size_t nq,
  size_t id_v, size_t nv,
  bool is_odom) {
  auto jsp = std::make_shared<ros2::JointStateProvider>(name,
                                                        id_q, nq,
                                                        id_v, nv);
  huron_node_->AddJointStateProvider(jsp, topic, nq, nv, is_odom);
  return jsp;
}

std::shared_ptr<huron::ForceTorqueSensor>
Ros2Environment::CreateForceTorqueSensor(
  const std::string& name,
  const std::string& topic,
  bool reverse_wrench_direction,
  std::weak_ptr<const multibody::Frame> frame) {
  auto fts = std::make_shared<ros2::ForceTorqueSensor>(
      name, reverse_wrench_direction, frame);
  huron_node_->AddForceTorqueSensor(fts, topic);
  return fts;
}

std::shared_ptr<huron::MovingInterface>
Ros2Environment::CreateJointGroupController(
  const std::string& topic,
  size_t dim) {
  auto jgc = std::make_shared<ros2::JointGroupController>(dim);
  huron_node_->AddJointGroupController(jgc, topic);
  return jgc;
}

}  // namespace ros2
}  // namespace huron
