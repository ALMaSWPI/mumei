# include <rclcpp/rclcpp.hpp>
# include "huron_ros2/robot.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<huron::ros2::HuronNode>());
  rclcpp::shutdown();

  return 0;
}
