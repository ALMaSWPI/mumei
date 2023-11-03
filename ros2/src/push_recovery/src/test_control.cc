#include "../include/push_recovery/control.h"
int main() {
  std::cout << "This is meant to use for testing";
  PushRecoveryControl Ibrahim;
  Eigen::VectorXd v(3);
  v << 1, 2, 3;
  std::cout << "v2 =" << std::endl << v(2) << std::endl;
  Ibrahim.r1_ft_force = {1,  2, 3};
  Ibrahim.r1_ft_torque = {1,  2, 3};
  Ibrahim.l1_ft_force = {1,  2, 3};
  Ibrahim.l1_ft_torque = {1,  2, 3};
  std::cout << "Printing =" << std::endl <<
    Ibrahim.GetTorque() << std::endl;
}
