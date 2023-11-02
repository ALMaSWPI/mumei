#include <rclcpp/rclcpp.hpp>
#include <huron/utils/time.h>
#include "huron_ros2/huron.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto huron_node = std::make_shared<huron::ros2::HuronNode>();
  // Instantiate a Huron object
  huron::ros2::Huron huron(huron_node);

  auto start = std::chrono::steady_clock::now();
  bool moved = false;
  
  while (rclcpp::ok()) {
    // Prints out joint states
    auto joint_position = huron.GetJointPosition();
    auto joint_velocity = huron.GetJointVelocity();
    std::cout << "Position: ";
    for (auto& p : joint_position) {
      std::cout << p << " ";
    }
    std::cout << std::endl;
    std::cout << "Velocity: ";
    for (auto& v : joint_velocity) {
      std::cout << v << " ";
    }
    std::cout << "\n\n";

    // After 10 seconds, move the joints
    if (since(start).count() > 10000 && !moved) {
      moved = true;
      huron.Move({10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
                   -10.5, -10.5, -10.5, -10.5, -10.5, -10.5});
    }

    huron.Loop();
  }
  rclcpp::shutdown();

  return 0;
}
