#include "robot.h"

int main(int argc, char* argv[]) {
  Robot* pcRobot = new Robot();
  pcRobot->Init();
  rclcpp::init(argc, argv); // argc, argv but I just set them to 0 here
  rclcpp::spin_some(std::make_shared<Robot>());
  rclcpp::shutdown();

}