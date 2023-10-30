#include <iostream>
#include <rclcpp.h>
using namespace std;

class PushRecoveryControl{
 private:
// EOM of 3 DOF model
// Mass in kg, length in meter
  float m1=5.9117,
        m2=4.2554,
        m3=10.19329;

  float lc1=0.15149,
        lc2=0.24517,
        lc3=0.1585;

  float l1=0.3715,
        l2=0.49478,
        l3=0.32662;

  float g=9.81;
  float I1=0.0222,
        I2=0.01009,
        I3=0.0219 ;

  //Desired position, velocity and acceleration of location of the com

  float theta1_d=0,
        theta2_d=0,
        theta3_d=0;

  float theta1_dot_d=0,
        theta2_dot_d=0,
        theta3_dot_d=0;

  float theta1_dddot=0,
        theta2_dddot=0,
        theta3_dddot=0;

  float x_com_d= 0,
        x_com_ddot = 0,
        x_com_dddot = 0;

  //Values
  float theta1, theta2, theta3 = 0;
  float theta1_dot, theta2_dot, theta3_dot = 0;
 public:
  void Init(){
    //Reading values from the model -> Assign into the values
        /**
         * theta1=ankle_pitch_theta;
          theta2=knee_pitch_theta;
          theta3=hip_pitch_theta;
          theta1_dot=ankle_pitch_theta_dot;
          theta2_dot=knee_pitch_theta_dot;
          theta3_dot=hip_pitch_theta_dot;
         */
  };
  void ReadFSR(){
    /**
     * % read the force sensor readings right foot
        r1_ft = receive(r1_ft_sensor);
        % read the force sensor readings left foot
        l1_ft = receive(l1_ft_sensor);
     */
  }

  void CalculateCOP(){
    /**
     * COP(end+1)=calcCOP(r1_ft, l1_ft) ;
      COP_filtered(end+1)=COP_filtered(end)*alpha + (1-alpha)*COP(end);
      cop = COP_filtered(end);
     */
  }

  //Note the assigning values:  q1 = theta1;
  // q2 = theta2;
  // q3 = theta3;
  // q_dot1 = theta1_dot;
  // q_dot2 = theta2_dot;
  // q_dot3 = theta3_dot;
  // r1 = lc1;
  // r2 = lc2;
  // r3 = lc3;

  void ModelCalculation(){
    // M = [I1 + I2 + I3 + l1^2*m2 + l1^2*m3 + l2^2*m3 + m1*r1^2 + m2*r2^2 + m3*r3^2 + 2*l1*m3*r3*cos(q2 + q3) + 2*l1*l2*m3*cos(q2) + 2*l1*m2*r2*cos(q2) + 2*l2*m3*r3*cos(q3), m3*l2^2 + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 + m2*r2^2 + l1*m2*cos(q2)*r2 + m3*r3^2 + l1*m3*cos(q2 + q3)*r3 + I2 + I3, I3 + m3*r3^2 + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3);
    //m3*l2^2 + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 + m2*r2^2 + l1*m2*cos(q2)*r2 + m3*r3^2 + l1*m3*cos(q2 + q3)*r3 + I2 + I3, m3*l2^2 + 2*m3*cos(q3)*l2*r3 + m2*r2^2 + m3*r3^2 + I2 + I3, m3*r3^2 + l2*m3*cos(q3)*r3 + I3;
    //I3 + m3*r3^2 + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3), m3*r3^2 + l2*m3*cos(q3)*r3 + I3, m3*r3^2 + I3];
    //
    // C = [- l1*m3*q_dot2^2*r3*sin(q2 + q3) - l1*m3*q_dot3^2*r3*sin(q2 + q3) - l1*l2*m3*q_dot2^2*sin(q2) - l1*m2*q_dot2^2*r2*sin(q2) - l2*m3*q_dot3^2*r3*sin(q3) - 2*l1*m3*q_dot1*q_dot2*r3*sin(q2 + q3) - 2*l1*m3*q_dot1*q_dot3*r3*sin(q2 + q3) - 2*l1*m3*q_dot2*q_dot3*r3*sin(q2 + q3) - 2*l1*l2*m3*q_dot1*q_dot2*sin(q2) - 2*l1*m2*q_dot1*q_dot2*r2*sin(q2) - 2*l2*m3*q_dot1*q_dot3*r3*sin(q3) - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3);
    //    l1*m3*q_dot1^2*r3*sin(q2 + q3) + l1*l2*m3*q_dot1^2*sin(q2) + l1*m2*q_dot1^2*r2*sin(q2) - l2*m3*q_dot3^2*r3*sin(q3) - 2*l2*m3*q_dot1*q_dot3*r3*sin(q3) - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3);
    //    l1*m3*q_dot1^2*r3*sin(q2 + q3) + l2*m3*q_dot1^2*r3*sin(q3) + l2*m3*q_dot2^2*r3*sin(q3) + 2*l2*m3*q_dot1*q_dot2*r3*sin(q3)];
    //
    // G = [- g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2) - g*l1*m2*sin(q1) - g*l1*m3*sin(q1) - g*m1*r1*sin(q1) - g*m3*r3*sin(q1 + q2 + q3);
    //    - g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2) - g*m3*r3*sin(q1 + q2 + q3);
    //    -g*m3*r3*sin(q1 + q2 + q3)];
    //
    // N = C + G;
  }
  void CalculateCOM(){
    //X_COM= -1*((lc1*sin(theta1))*m1 + (l1*sin(theta1)+lc2*sin(theta1+theta2))*m2 + (l1*sin(theta1)+l2*sin(theta1+theta2)+lc3*sin(theta1+theta2+theta3))*m3) / (m1+m2+m3); % Center of Mass position in x_direction
    //X_dot_COM= -1* (m1*(theta1_dot*lc1*cos(theta1)) + m2*(theta1_dot*l1*cos(theta1) +(theta1_dot+theta2_dot)*lc2*cos(theta1+theta2)) + m3*(theta1_dot*l1*cos(theta1)+(theta1_dot+theta2_dot)*l2*cos(theta1+theta2)+(theta1_dot+theta2_dot+theta3_dot)*lc3*cos(theta1+theta2+theta3)) )/(m1+m2+m3); % velocity of the COM in x_direction
    //J_X_COM= -1* [(m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1))/(m1 + m2 + m3), (m3*(l2*cos(theta1 + theta2) + lc3*cos(theta1 + theta2 + theta3)) + lc2*m2*cos(theta1 + theta2))/(m1 + m2 + m3), (lc3*m3*cos(theta1 + theta2 + theta3))/(m1 + m2 + m3)]; % Jacobian Matrix of X-COM Jx
    //J_X_COM_dot=-1*[(-m1*theta1_dot*lc1*sin(theta1) + m2*(-l1*theta1_dot*sin(theta1)-(theta1_dot+theta2_dot)*lc2*sin(theta1+theta2) )  + m3*(-l2*(theta1_dot+theta2_dot)*sin(theta1+theta2)-l1*theta1_dot*sin(theta1)-(theta1_dot+theta2_dot+theta3_dot)*lc3*sin(theta1+theta2+theta3)) )/(m1+m2+m3) ,( m3*(-l2*(theta1_dot+theta2_dot)*sin(theta1+theta2) -lc3*(theta1_dot+theta2_dot+theta3_dot)*sin(theta1+theta2+theta3) ) -lc2*(theta1_dot+theta2_dot)*m2*sin(theta1+theta2) )/(m1+m2+m3) ,(-lc3*(theta1_dot+theta2_dot+theta3_dot)*m3*sin(theta1+theta2+theta3) )/(m1+m2+m3)]; % Time Derivative of Jacobian Matrix
    //Pseudo_J_X_COM=pinv(J_X_COM); % Pseudo Inverse of J_X_COM
    //
    //Z_COM=( (lc1*cos(theta1))*m1 + (l1*cos(theta1)+lc2*cos(theta1+theta2))*m2 + (l1*cos(theta1)+l2*cos(theta1+theta2)+lc3*cos(theta1+theta2+theta3))*m3  ) / (m1+m2+m3); % Center of Mass position in z_direction (desired is 0.6859)
    //Z_dot_COM=(  -m1*(theta1_dot*lc1*sin(theta1)) + m2*(-theta1_dot*l1*sin(theta1) - (theta1_dot+theta2_dot)*lc2*sin(theta1+theta2) ) + m3*(-theta1_dot*l1*sin(theta1) - (theta1_dot+theta2_dot)*l2*sin(theta1+theta2) - (theta1_dot+theta2_dot+theta3_dot)*lc3*sin(theta1+theta2+theta3)) )/(m1+m2+m3); % velocity of the COM in z_direction
    //J_Z_COM=[(m3*(-l2*sin(theta1 + theta2) - l1*sin(theta1) - lc3*sin(theta1 + theta2 + theta3)) + m2*(-lc2*sin(theta1 + theta2) - l1*sin(theta1)) - lc1*m1*sin(theta1))/(m1 + m2 + m3), (m3*(-l2*sin(theta1 + theta2) - lc3*sin(theta1 + theta2 + theta3)) - lc2*m2*sin(theta1 + theta2))/(m1 + m2 + m3), (-lc3*m3*sin(theta1 + theta2 + theta3))/(m1 + m2 + m3)]; % Jacobian Matrix of Z-COM Jz
    //
    //J_COM=[J_X_COM ; J_Z_COM]; % Linear part of Jacobian matrix of COM
//    //Angular position and velocity of COM
//    Angle_COM=-( theta1 + theta2 + theta3 );
//    W_COM=-(theta1_dot+theta2_dot+theta3_dot);
//
//
//    J_W_COM=[1 , 1 ,1];
//
//    J_total_COM=[J_X_COM; J_W_COM] ; % linear and angular part of Jacobian of COM
//        J_total_COM_dot=[ J_X_COM_dot; 0 0 0]; % time derivative of total Jacobian of COM
//        J_total_COM_pseduo=pinv(J_total_COM);  % pseduo inverse of the total Jacobian of COM

  }
  void CalculateAngularCOM(){

//    Angle_COM_calculated=angle((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3) - ((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))*1i)/(m1 + m2 + m3)) ;
//    W_COM_calculated=-((real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2*((imag((m3*(l2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot + lc3*sin(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot) + lc1*m1*sin(theta1)*theta1_dot)/(m1 + m2 + m3)) + real((m3*(l2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot + lc3*cos(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot) + lc1*m1*cos(theta1)*theta1_dot)/(m1 + m2 + m3)))/(real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3))) - ((imag((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) - real((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))*(real((m3*(l2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot + lc3*sin(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*sin(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*sin(theta1)*theta1_dot) + lc1*m1*sin(theta1)*theta1_dot)/(m1 + m2 + m3)) - imag((m3*(l2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot + lc3*cos(theta1 + theta2 + theta3)*(theta1_dot + theta2_dot + theta3_dot)) + m2*(lc2*cos(theta1 + theta2)*(theta1_dot + theta2_dot) + l1*cos(theta1)*theta1_dot) + lc1*m1*cos(theta1)*theta1_dot)/(m1 + m2 + m3))))/(real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2))/((real((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) + imag((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2 + (imag((m2*(l1*cos(theta1) + lc2*cos(theta1 + theta2)) + m3*(l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3) + l2*cos(theta1 + theta2)) + lc1*m1*cos(theta1))/(m1 + m2 + m3)) - real((m3*(l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3) + l2*sin(theta1 + theta2)) + m2*(l1*sin(theta1) + lc2*sin(theta1 + theta2)) + lc1*m1*sin(theta1))/(m1 + m2 + m3)))^2);
//    Theta_COM_calculated(end+1)=Angle_COM_calculated;
//    W_velocity_COM_calculated(end+1)=W_COM_calculated;

  }
  void SMCController(){
    //% SMC for Linear motion rate of change of linear momentum
    //
    //error_in_x=X_COM;       error_dot_in_x=X_dot_COM;
    //
    //theta_dot = [theta1_dot; theta2_dot; theta3_dot];
    //
    //L11=1.9; k11=-1; p11=1; c11=7; Q11=0.001; a11=2; z11=2.5; % after editing the first term/first approach
    //
    //s_linear_motion = error_dot_in_x + L11*error_in_x;
    //s_linear_momentum_controller(end+1)=s_linear_motion;
    //
    //f11= c11 * ( 1 - exp( k11*(abs(s_linear_motion))^p11 ) );
    //s_linear_motion_dot= -Q11 * ( (abs(s_linear_motion))^f11 ) *sign(s_linear_motion) - z11 * (abs(s_linear_motion))^a11  * s_linear_motion ;
    //
    //% Angular Momentum control part
    //A_theta =[I1 + I2 + I3 - (m3*(l2*sin(theta1 + theta2) + l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3)) + m2*(lc2*sin(theta1 + theta2) + l1*sin(theta1)) + lc1*m1*sin(theta1))^2/(m1 + m2 + m3) + l1^2*m2 + l1^2*m3 + l2^2*m3 + lc1^2*m1 + lc2^2*m2 + lc3^2*m3 - (m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1))^2/(m1 + m2 + m3) + 2*l1*lc3*m3*cos(theta2 + theta3) + 2*l1*l2*m3*cos(theta2) + 2*l1*lc2*m2*cos(theta2) + 2*l2*lc3*m3*cos(theta3), I2 + I3 + l2^2*m3 + lc2^2*m2 + lc3^2*m3 - ((m3*(l2*sin(theta1 + theta2) + l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3)) + m2*(lc2*sin(theta1 + theta2) + l1*sin(theta1)) + lc1*m1*sin(theta1))*(lc3*m3*sin(theta1 + theta2 + theta3) + l2*m3*sin(theta1 + theta2) + lc2*m2*sin(theta1 + theta2)))/(m1 + m2 + m3) - ((lc3*m3*cos(theta1 + theta2 + theta3) + l2*m3*cos(theta1 + theta2) + lc2*m2*cos(theta1 + theta2))*(m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1)))/(m1 + m2 + m3) + l1*lc3*m3*cos(theta2 + theta3) + l1*l2*m3*cos(theta2) + l1*lc2*m2*cos(theta2) + 2*l2*lc3*m3*cos(theta3), I3 + lc3^2*m3 + l1*lc3*m3*cos(theta2 + theta3) + l2*lc3*m3*cos(theta3) - (lc3*m3*cos(theta1 + theta2 + theta3)*(m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1)))/(m1 + m2 + m3) - (lc3*m3*sin(theta1 + theta2 + theta3)*(m3*(l2*sin(theta1 + theta2) + l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3)) + m2*(lc2*sin(theta1 + theta2) + l1*sin(theta1)) + lc1*m1*sin(theta1)))/(m1 + m2 + m3)];
    //
    //A_theta_pseudo = [(I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + l1^2*m1*m2 + l1^2*m1*m3 + l2^2*m1*m3 + l2^2*m2*m3 + lc1^2*m1*m2 + lc1^2*m1*m3 + lc2^2*m1*m2 + lc2^2*m2*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3 - 2*l2*lc2*m2*m3 + 2*l1*lc3*m1*m3*cos(theta2 + theta3) - 2*lc1*lc3*m1*m3*cos(theta2 + theta3) + 2*l1*l2*m1*m3*cos(theta2) + 2*l1*lc2*m1*m2*cos(theta2) - 2*l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - 2*lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))/((m1 + m2 + m3)*((I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + l1^2*m1*m2 + l1^2*m1*m3 + l2^2*m1*m3 + l2^2*m2*m3 + lc1^2*m1*m2 + lc1^2*m1*m3 + lc2^2*m1*m2 + lc2^2*m2*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3 - 2*l2*lc2*m2*m3 + 2*l1*lc3*m1*m3*cos(theta2 + theta3) - 2*lc1*lc3*m1*m3*cos(theta2 + theta3) + 2*l1*l2*m1*m3*cos(theta2) + 2*l1*lc2*m1*m2*cos(theta2) - 2*l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - 2*lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))^2/(m1 + m2 + m3)^2 + (I3*m1 + I3*m2 + I3*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l2*lc3*m1*m3*cos(theta3) + l2*lc3*m2*m3*cos(theta3) - lc2*lc3*m2*m3*cos(theta3))^2/(m1 + m2 + m3)^2 + (I2*m1 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + l2^2*m1*m3 + l2^2*m2*m3 + lc2^2*m1*m2 + lc2^2*m2*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 - 2*l2*lc2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l1*l2*m1*m3*cos(theta2) + l1*lc2*m1*m2*cos(theta2) - l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))^2/(m1 + m2 + m3)^2));                                                                                                                               (I2*m1 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + l2^2*m1*m3 + l2^2*m2*m3 + lc2^2*m1*m2 + lc2^2*m2*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 - 2*l2*lc2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l1*l2*m1*m3*cos(theta2) + l1*lc2*m1*m2*cos(theta2) - l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))/((m1 + m2 + m3)*((I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + l1^2*m1*m2 + l1^2*m1*m3 + l2^2*m1*m3 + l2^2*m2*m3 + lc1^2*m1*m2 + lc1^2*m1*m3 + lc2^2*m1*m2 + lc2^2*m2*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3 - 2*l2*lc2*m2*m3 + 2*l1*lc3*m1*m3*cos(theta2 + theta3) - 2*lc1*lc3*m1*m3*cos(theta2 + theta3) + 2*l1*l2*m1*m3*cos(theta2) + 2*l1*lc2*m1*m2*cos(theta2) - 2*l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - 2*lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))^2/(m1 + m2 + m3)^2 + (I3*m1 + I3*m2 + I3*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l2*lc3*m1*m3*cos(theta3) + l2*lc3*m2*m3*cos(theta3) - lc2*lc3*m2*m3*cos(theta3))^2/(m1 + m2 + m3)^2 + (I2*m1 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + l2^2*m1*m3 + l2^2*m2*m3 + lc2^2*m1*m2 + lc2^2*m2*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 - 2*l2*lc2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l1*l2*m1*m3*cos(theta2) + l1*lc2*m1*m2*cos(theta2) - l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))^2/(m1 + m2 + m3)^2));                                                                                                                                                                                                                                                                                                                                                (I3*m1 + I3*m2 + I3*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l2*lc3*m1*m3*cos(theta3) + l2*lc3*m2*m3*cos(theta3) - lc2*lc3*m2*m3*cos(theta3))/((m1 + m2 + m3)*((I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + l1^2*m1*m2 + l1^2*m1*m3 + l2^2*m1*m3 + l2^2*m2*m3 + lc1^2*m1*m2 + lc1^2*m1*m3 + lc2^2*m1*m2 + lc2^2*m2*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3 - 2*l2*lc2*m2*m3 + 2*l1*lc3*m1*m3*cos(theta2 + theta3) - 2*lc1*lc3*m1*m3*cos(theta2 + theta3) + 2*l1*l2*m1*m3*cos(theta2) + 2*l1*lc2*m1*m2*cos(theta2) - 2*l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - 2*lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))^2/(m1 + m2 + m3)^2 + (I3*m1 + I3*m2 + I3*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l2*lc3*m1*m3*cos(theta3) + l2*lc3*m2*m3*cos(theta3) - lc2*lc3*m2*m3*cos(theta3))^2/(m1 + m2 + m3)^2 + (I2*m1 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + l2^2*m1*m3 + l2^2*m2*m3 + lc2^2*m1*m2 + lc2^2*m2*m3 + lc3^2*m1*m3 + lc3^2*m2*m3 - 2*l2*lc2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l1*l2*m1*m3*cos(theta2) + l1*lc2*m1*m2*cos(theta2) - l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))^2/(m1 + m2 + m3)^2)) ];
    //
    //A_theta_dot = [-(2*l1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3) + 2*l1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3) - 2*lc1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3) - 2*lc1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3) + 2*l1*l2*m1*m3*theta2_dot*sin(theta2) + 2*l1*lc2*m1*m2*theta2_dot*sin(theta2) - 2*l2*lc1*m1*m3*theta2_dot*sin(theta2) + 2*l2*lc3*m1*m3*theta3_dot*sin(theta3) + 2*l2*lc3*m2*m3*theta3_dot*sin(theta3) - 2*lc1*lc2*m1*m2*theta2_dot*sin(theta2) - 2*lc2*lc3*m2*m3*theta3_dot*sin(theta3))/(m1 + m2 + m3), -(l1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3) + l1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3) - lc1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3) - lc1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3) + l1*l2*m1*m3*theta2_dot*sin(theta2) + l1*lc2*m1*m2*theta2_dot*sin(theta2) - l2*lc1*m1*m3*theta2_dot*sin(theta2) + 2*l2*lc3*m1*m3*theta3_dot*sin(theta3) + 2*l2*lc3*m2*m3*theta3_dot*sin(theta3) - lc1*lc2*m1*m2*theta2_dot*sin(theta2) - 2*lc2*lc3*m2*m3*theta3_dot*sin(theta3))/(m1 + m2 + m3), -(lc3*m3*(l1*m1*theta2_dot*sin(theta2 + theta3) + l1*m1*theta3_dot*sin(theta2 + theta3) - lc1*m1*theta2_dot*sin(theta2 + theta3) - lc1*m1*theta3_dot*sin(theta2 + theta3) + l2*m1*theta3_dot*sin(theta3) + l2*m2*theta3_dot*sin(theta3) - lc2*m2*theta3_dot*sin(theta3)))/(m1 + m2 + m3)];
    //
    //% % cop = COP(end);
    //
    //% first approach : desired angular momentum
    //% k_tunning =0.01; % for first option
    //% Desired_Angular_Momentum = k_tunning * ( (cop) - X_COM ) *  ( (m1+m2+m3) * (s_linear_motion_dot - ( L11 * error_dot_in_x ) ) );
    //k_tunning =2; % for second option
    //Desired_Angular_Momentum = k_tunning * ( (cop) - X_COM ) *  ( 1 ); % second option
    //% Desired accleration to achieve both linear and angular tasks without LPF
    //Desired_Theta_ddot =  (pinv( [ J_X_COM; A_theta  ])) *  ( [ ( s_linear_motion_dot - L11 * error_dot_in_x - J_X_COM_dot * theta_dot ) ; ( Desired_Angular_Momentum - (A_theta_dot * theta_dot )) ] ) ;
    //Torque_SMC_Linear_plus_angular_compensation =  M *  [ Desired_Theta_ddot(1,1) ; Desired_Theta_ddot(2,1) ; Desired_Theta_ddot(3,1) ]  +  N ;
  }
  void SMCPOstureCorrection(){
    //theta = [theta1; theta2; theta3]; % 3x1
    //theta_dot = [theta1_dot; theta2_dot; theta3_dot]; % 3x1
    //error_in_q = theta  ;
    //errordot_in_q = theta_dot ;
    //
    //% % for 80 N force
    //% lamda_of_q =diag ( [6.1 6.1 6.1] ); % 3x3
    //% k_of_q =diag (     [1.95 1.95 1.95]); % 3x3
    //% w1=1  ; w2=1 ; w3=1;
    //% %
    //
    //% for above 80 N
    //lamda_of_q =diag ( [2.8 2.8 2.8] ); % 3x3
    //k_of_q =diag (     [3 3 3]); % 3x3
    //w1=1  ; w2=1 ; w3=1;
    //
    //
    //s_of_q = errordot_in_q + lamda_of_q * error_in_q ; % 3x1
    //s_q1(end+1)=s_of_q(1,1);
    //s_q2(end+1)=s_of_q(2,1);
    //s_q3(end+1)=s_of_q(3,1);
    //
    //phi1 =0.002 ;
    //phi2 =0.02 ;
    //phi3 =0.02;
    //
    //if norm(s_of_q(1,1)) >= phi1
    //    sat1 = sign(s_of_q(1,1));
    //else
    //    sat1 = s_of_q(1,1) / phi1;
    //end
    //
    //if norm(s_of_q(2,1)) >= phi2
    //    sat2 = sign(s_of_q(2,1));
    //else
    //    sat2 = s_of_q(2,1) / phi2;
    //end
    //
    //if norm(s_of_q(3,1)) >= phi3
    //    sat3 = sign(s_of_q(3,1));
    //else
    //    sat3 = s_of_q(3,1) / phi3;
    //end
    //
    //saturation_function = [ sat1 ; sat2 ; sat3 ] ;
    //q_double_dot1 =  (-k_of_q(1,1) * ( abs(s_of_q(1,1))^w1  ) * sat1 )  - (lamda_of_q(1,1) * errordot_in_q(1,1)) ; % constant power rate reaching law
    //q_double_dot2 =  (-k_of_q(2,2) * ( abs(s_of_q(2,1))^w2  ) * sat2 )  - (lamda_of_q(2,1) * errordot_in_q(2,1)) ; % constant power rate reaching law
    //q_double_dot3 =  (-k_of_q(3,3) * ( abs(s_of_q(3,1))^w3  ) * sat3 )  - (lamda_of_q(3,1) * errordot_in_q(3,1)) ; % constant power rate reaching law
    //q_double_dot = [ q_double_dot1 ; q_double_dot2 ; q_double_dot3 ];
    //
    //Phi_N_of_q = (eye(3) - pinv( [ J_X_COM  ])*[ J_X_COM ])* q_double_dot; % second approach for angular momentum
    //T_posture_of_q = M* Phi_N_of_q  ;
  }
  void GetTorque(){
    //T = Torque_SMC_Linear_plus_angular_compensation + T_posture_of_q ;
    //
    //% if T(1) >= 30
    //%     T(1) = 30;
    //% end
    //% if T(1) <= -11.76
    //%     T(1) = -11.76;
    //% end
    //
    //T_ankle = T(1);
    //T_knee = T(2);
    //T_hip = T(3);
  }

};

int main()
{
  cout << "This is meant to use for testing" ;

}