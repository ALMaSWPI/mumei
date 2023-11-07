#include "push_recovery/control.h"
#include "huron/utils/time.h"
#include "huron_ros2/huron.h"
#include <rclcpp/rclcpp.hpp>


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
    auto fsr_left = huron.GetForceResistorSensorLeft();
    auto fsr_right = huron.GetForceResistorSensorRight();
//    std::cout << "FSR: ";
//    for (auto& p : fsr_left) {
//      std::cout << p << " ";
//    }
//    std::cout << std::endl;
//    std::cout << "Velocity: ";
//    for (auto& v : joint_velocity) {
//      std::cout << v << " ";
//    }
//    std::cout << "\n\n";

    // After 10 seconds, move the joints
    if (since(start).count() > 10000 && !moved) {
      moved = true;
    }
    if (moved) {
      std::cout << "Calculating new torque";

      PushRecoveryControl Ibrahim;
      Ibrahim.r1_ft_force = {fsr_right.at(0),
                             fsr_right.at(1), fsr_right.at(2)};
      Ibrahim.r1_ft_torque = {fsr_right.at(3),
                              fsr_right.at(4), fsr_right.at(5)};
      Ibrahim.l1_ft_force = {fsr_left.at(0),
                             fsr_left.at(1), fsr_left.at(2)};
      Ibrahim.l1_ft_torque = {fsr_left.at(3),
                              fsr_left.at(4), fsr_left.at(5)};

      Eigen::MatrixXf Torque(3, 1);
      Torque = Ibrahim.GetTorque();
      std::cout << "Publishing =" << std::endl <<
        Torque << std::endl;

//      tau_msg.Data = [0, 0, T_hip, T_knee, T_ankle, 0, ...
//                      0, 0, T_hip, T_knee, T_ankle, 0];
      huron.Move({0,
                  0,
                  Torque(0, 0),
                  Torque(1, 0),
                  Torque(2, 0),
                  0,
                  0,
                  0,
                  Torque(0, 0),
                  Torque(1, 0),
                  Torque(2, 0),
                  0
      });
    }


    huron.Loop();
  }
  rclcpp::shutdown();

  return 0;
}
