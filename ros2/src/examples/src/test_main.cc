#include <rclcpp/rclcpp.hpp>
#include <huron_ros2/huron.h>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  while (rclcpp::ok()) {
    auto huron_node = std::make_shared<huron::ros2::HuronNode>();
    // Instantiate a Huron object
    huron::ros2::Huron huron(huron_node);

    rclcpp::spin_some(huron_node);
  }
  rclcpp::shutdown();

  return 0;
}
