#include <iostream>

#include "huron/control_interfaces/legged_robot.h"
#include "huron/control_interfaces/sensor.h"
#include "huron/control_interfaces/constant_state_provider.h"
#include "huron/driver/can/socket_can_bus.h"
#include "huron/odrive/odrive_rotary_encoder.h"
#include "huron/odrive/odrive_torque_motor.h"
#include "huron/utils/time.h"

const double kGearRatio = 1.0;
const double kCPR = 4096.0;

const std::array<std::string, 8> joints_without_encoder = {
  "l_hip_yaw_joint",
  "l_hip_roll_joint",
  "l_ankle_pitch_joint",
  "r_hip_roll_joint",
  "l_ankle_roll_joint",
  "r_ankle_pitch_joint",
  "r_hip_yaw_joint",
  "r_ankle_roll_joint"
};

// Helper function to print vectors
template<typename T>
void PrintVector(const std::vector<T>& vec) {
  for (auto& e : vec) {
    std::cout << e << ' ';
  }
  std::cout << '\n';
}

class Huron : public huron::LeggedRobot {
 public:
  Huron() = default;
  Huron(const Huron&) = delete;
  Huron& operator=(const Huron&) = delete;
  ~Huron() override = default;

  void Initialize() override {
    // Initialize all motors
    for (auto& component : moving_components_) {
      auto motor = std::dynamic_pointer_cast<huron::TorqueMotor>(
        component);
      if (motor != nullptr) {
        motor->Initialize();
      }
    }
    // Initialize all sensors
    for (auto& sp : non_joint_state_providers_) {
      auto sensor = std::dynamic_pointer_cast<huron::Sensor>(sp);
      if (sensor != nullptr) {
        sensor->Initialize();
      }
    }
    for (auto& sp : joint_state_providers_) {
      auto sensor = std::dynamic_pointer_cast<huron::Sensor>(sp);
      if (sensor != nullptr) {
        sensor->Initialize();
      }
    }
  }
  void SetUp() override {
    // Initialize all motors
    for (auto& component : moving_components_) {
      auto motor = std::dynamic_pointer_cast<huron::TorqueMotor>(
        component);
      if (motor != nullptr) {
        motor->SetUp();
      }
    }
    // Initialize all sensors
    for (auto& sp : non_joint_state_providers_) {
      auto sensor = std::dynamic_pointer_cast<huron::Sensor>(sp);
      if (sensor != nullptr) {
        sensor->SetUp();
      }
    }
    for (auto& sp : joint_state_providers_) {
      auto sensor = std::dynamic_pointer_cast<huron::Sensor>(sp);
      if (sensor != nullptr) {
        sensor->SetUp();
      }
    }
  }
  void Terminate() override {
    // Initialize all motors
    for (auto& component : moving_components_) {
      auto motor = std::dynamic_pointer_cast<huron::TorqueMotor>(
        component);
      if (motor != nullptr) {
        motor->Terminate();
      }
    }
    // Initialize all sensors
    for (auto& sp : non_joint_state_providers_) {
      auto sensor = std::dynamic_pointer_cast<huron::Sensor>(sp);
      if (sensor != nullptr) {
        sensor->Terminate();
      }
    }
    for (auto& sp : joint_state_providers_) {
      auto sensor = std::dynamic_pointer_cast<huron::Sensor>(sp);
      if (sensor != nullptr) {
        sensor->Terminate();
      }
    }
  }
};

int main(int argc, char* argv[]) {
  Huron robot;

  huron::driver::can::SocketCanBus hcb0{"can0", 0};
  huron::driver::can::SocketCanBus hcb1{"can0", 1};
  huron::driver::can::SocketCanBus hcb6{"can1", 6};
  huron::driver::can::SocketCanBus hcb7{"can1", 7};
  auto left_knee_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb0, 0, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());
  auto left_hip_pitch_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb1, 1, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());
  auto right_knee_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb6, 6, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());
  auto right_hip_pitch_odrive = std::make_shared<huron::odrive::ODriveCAN>(
    &hcb7, 7, std::make_unique<huron::odrive::ODrive::ODriveConfiguration>());

  robot.GetModel()->AddModelImpl(
    huron::multibody::ModelImplType::kPinocchio, true);
  robot.GetModel()->BuildFromUrdf("huron.urdf",
                                  huron::multibody::JointType::kFreeFlyer);

  // Configure joints
  // root_joint
  Eigen::Vector<double, 13> floating_base_state;
  floating_base_state << 0.0, 0.0, 1.123, 0.0, 0.0, 0.0, 1.0,  // positions
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // velocities
  auto floating_joint_sp =
    std::make_shared<huron::ConstantStateProvider>(floating_base_state);
  robot.RegisterStateProvider(floating_joint_sp, true);
  robot.GetModel()->SetJointStateProvider(1, floating_joint_sp);

  // 4 encoders
  auto left_knee_encoder = std::make_shared<huron::odrive::ODriveEncoder>(
    kGearRatio, kCPR, left_knee_odrive);
  auto left_hip_pitch_encoder = std::make_shared<huron::odrive::ODriveEncoder>(
    kGearRatio, kCPR, left_hip_pitch_odrive);
  auto right_knee_encoder = std::make_shared<huron::odrive::ODriveEncoder>(
    kGearRatio, kCPR, right_knee_odrive);
  auto right_hip_pitch_encoder = std::make_shared<huron::odrive::ODriveEncoder>(
    kGearRatio, kCPR, right_hip_pitch_odrive);

  robot.RegisterStateProvider(left_knee_encoder, true);
  robot.GetModel()->SetJointStateProvider(
    robot.GetModel()->GetJointIndex("l_knee_pitch_joint"),
    left_knee_encoder);
  robot.RegisterStateProvider(left_hip_pitch_encoder, true);
  robot.GetModel()->SetJointStateProvider(
    robot.GetModel()->GetJointIndex("l_hip_pitch_joint"),
    left_hip_pitch_encoder);
  robot.RegisterStateProvider(right_knee_encoder, true);
  robot.GetModel()->SetJointStateProvider(
    robot.GetModel()->GetJointIndex("r_knee_pitch_joint"),
    right_knee_encoder);
  robot.RegisterStateProvider(right_hip_pitch_encoder, true);
  robot.GetModel()->SetJointStateProvider(
    robot.GetModel()->GetJointIndex("r_hip_pitch_joint"),
    right_hip_pitch_encoder);

  // Use constant state provider for the remaining joints
  for (const auto& joint_name : joints_without_encoder) {
    auto sp =
      std::make_shared<huron::ConstantStateProvider>(Eigen::Vector2d::Zero());
    robot.RegisterStateProvider(sp, true);
    robot.GetModel()->SetJointStateProvider(
      robot.GetModel()->GetJointIndex(joint_name),
      sp);
  }

  robot.GetModel()->Finalize();

  // 4 motors
  robot.AddToGroup(
    std::make_shared<huron::odrive::TorqueMotor>(
      left_knee_odrive));
  robot.AddToGroup(
    std::make_shared<huron::odrive::TorqueMotor>(
      left_hip_pitch_odrive));
  robot.AddToGroup(
    std::make_shared<huron::odrive::TorqueMotor>(
      right_knee_odrive));
  robot.AddToGroup(
    std::make_shared<huron::odrive::TorqueMotor>(
      right_hip_pitch_odrive));

  // Initialize
  std::cout << "Initializing..." << std::endl;
  robot.Initialize();
  std::cout << "Initialization completed." << std::endl;

  std::cout << "Enabling motor..." << std::endl;
  robot.SetUp();
  std::cout << "Motor enabled." << std::endl;

  std::cout << "Initial position: \n"
            << robot.GetJointPositions() << std::endl;

  std::cout << "Moving..." << std::endl;
  auto start_time = std::chrono::steady_clock::now();

  robot.Move(Eigen::Vector4d{0.2, 0.2, 0.2, 0.2});

  while (since(start_time).count() < 3000 /* ms */) {
    robot.UpdateJointStates();
    std::cout << "Current positions: \n"
              << robot.GetJointPositions().transpose() << std::endl;

    std::cout << "Current velocities: \n"
              << robot.GetJointVelocities().transpose() << std::endl;
  }

  std::cout << "Stopping..." << std::endl;
  robot.Stop();

  std::cout << "Final position: \n"
            << robot.GetJointPositions().transpose() << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(5));

  robot.Terminate();
  std::cout << "Terminated." << std::endl;

  return 0;
}

