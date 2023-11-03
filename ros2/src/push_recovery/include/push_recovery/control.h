#include <iostream>
#include <eigen3/Eigen/Dense>
#include <complex>
#include <vector>
std::complex<double> i(0.0, 1.0);

class PushRecoveryControl {
 private:
  // EOM of 3 DOF model
  // Mass in kg, length in meter
  float alpha = 0.7;
  float m1 = 5.9117,
        m2 = 4.2554,
        m3 = 10.19329;

  float lc1 = 0.15149,
        lc2 = 0.24517,
        lc3 = 0.1585;

  float l1 = 0.3715,
        l2 = 0.49478,
        l3 = 0.32662;

  float g = 9.81;
  float I1 = 0.0222,
        I2 = 0.01009,
        I3 = 0.0219;

  // Desired position, velocity and acceleration of location of the com

  float theta1_d = 0,
        theta2_d = 0,
        theta3_d = 0;

  float theta1_dot_d = 0,
        theta2_dot_d = 0,
        theta3_dot_d = 0;

  float theta1_dddot = 0,
        theta2_dddot = 0,
        theta3_dddot = 0;

  float x_com_d = 0,
        x_com_ddot = 0,
        x_com_dddot = 0;

  // Values
  float theta1, theta2, theta3 = 0;
  float theta1_dot, theta2_dot, theta3_dot = 0;
  float X_COM, X_dot_COM = 0;

 public:
  std::vector<float> r1_ft_torque, l1_ft_torque, r1_ft_force, l1_ft_force;
  float CalculateXCOP();


  Eigen::MatrixXf ModelCalculation();

  Eigen::MatrixXf CalculateCOM();

  template <typename T>
  int sign (const T &val) { return (val > 0) - (val < 0); }

  Eigen::MatrixXf SMCController(Eigen::RowVectorXf cop,
                                Eigen::MatrixXf J_X_COM,
                                Eigen::MatrixXf J_X_COM_dot);

  Eigen::MatrixXf SMCPOstureCorrection();

  Eigen::MatrixXf GetTorque();
};
