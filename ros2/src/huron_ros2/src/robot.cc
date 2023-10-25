# include "huron_ros2/robot.h"

namespace huron {
namespace ros2 {
void Robot::Initialize(){
  rclcpp::init(0, 0); // argc, argv but I just set them to 0 here
  rclcpp::spin(std::make_shared<HuronNode>());
  rclcpp::shutdown(); //TODO: verify if this should be here
}


void Robot::Terminate(){
  rclcpp::shutdown();
}

bool Robot::Move(std::vector<float> values){
  // Publishing to the topic
  auto message = sensor_msgs::msg::JointState();
  for (int i = 0; i < 3; i++){
    message->position[i] = values[i];
  }
  RCLCPP_INFO(this->get_logger(), "Publishing Position: '%s'", message.data.c_str());
  HuronNode.publisher_joints_->publish(message);
  return True;
}

bool Robot::MoveVelocity(std::vector<float> values){
  // Publishing to the topic
  auto message = sensor_msgs::msg::JointState();
  for (int i = 0; i < 3; i++){
    message->velocity[i] = values[i];
  }
  RCLCPP_INFO(this->get_logger(), "Publishing Velocity: '%s'", message.data.c_str());
  HuronNode.publisher_joints_->publish(message);
  return True;
}

bool Robot::MoveTorque(std::vector<float> values){
  // Publishing to the topic
  auto message = sensor_msgs::msg::JointState();
  for (int i = 0; i < 3; i++){
    message->effort[i] = values[i];
  }
  RCLCPP_INFO(this->get_logger(), "Publishing Torque: '%s'", message.data.c_str());
  HuronNode.publisher_joints_->publish(message);
  return True;
}

bool Robot::Stop(){
  // Publishing to the topic
  auto message = sensor_msgs::msg::JointState();
  for (int i = 0; i < 3; i++){
    message->effort[i] = 0;
    message->velocity[i] = 0;
  }
  RCLCPP_INFO(this->get_logger(), "Terminated'", message.data.c_str());
  HuronNode.publisher_joints_->publish(message);
  return True;
}


}
}
